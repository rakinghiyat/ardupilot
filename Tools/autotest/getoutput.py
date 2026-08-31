#!/usr/bin/env python3
"""
sync_and_analyze_servo_v2.py

Versi disesuaikan dengan output CSV JSBSim "sakti.csv" yang sudah
berisi Hinge_torque_*_ftlb langsung (dihitung di JSBSim, tidak perlu
dihitung ulang di Python).

Cara pakai:
  pip install pymavlink pandas numpy matplotlib 
  pip install pymavlink pandas numpy --break-system-packages
  python3 getoutput.py --bin log_ardupilot.bin --csv sakti.csv --out merged_analysis.csv
"""

import argparse
import sys
import numpy as np
import pandas as pd

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink belum terinstall. Jalankan:")
    print("  pip install pymavlink --break-system-packages")
    sys.exit(1)

SAFETY_MARGIN = 2.0   # margin lebih besar karena Cm_hinge masih estimasi kasar
FT_LB_TO_NM = 1.35582

SURFACES = ("LO", "RO", "LI", "RI")

# Mapping nama surface internal kita -> prefix nama kolom JSBSim CSV asli
# (left-outboard, right-outboard, left-inboard, right-inboard)
JSB_NAME_MAP = {
    "LO": "left-outboard",
    "RO": "right-outboard",
    "LI": "left-inboard",
    "RI": "right-inboard",
}


# ---------------------------------------------------------
# 1. BACA LOG ARDUPILOT (.bin) -> ekstrak message SVOD
# ---------------------------------------------------------
def read_ardupilot_svod(bin_path: str) -> pd.DataFrame:
    print(f"[INFO] Membaca dataflash log: {bin_path}")
    mlog = mavutil.mavlink_connection(bin_path, dialect="ardupilotmega")

    rows = []
    while True:
        msg = mlog.recv_match(type="SVOD", blocking=False)
        if msg is None:
            break
        rows.append({
            "TimeUS": msg.TimeUS,
            "Frame":  msg.Frame,
            "LO_deg_ap": msg.LO,
            "RO_deg_ap": msg.RO,
            "LI_deg_ap": msg.LI,
            "RI_deg_ap": msg.RI,
            "Thr_ap":    msg.Thr,
        })

    if not rows:
        print("[WARNING] Tidak ada message 'SVOD' ditemukan di log.")
        sys.exit(1)

    df = pd.DataFrame(rows)
    df["time_s_ap"] = df["TimeUS"] * 1e-6
    print(f"[INFO] {len(df)} baris SVOD terbaca dari ArduPilot log.")
    print(f"[INFO] Range Frame ArduPilot : {df['Frame'].min()} - {df['Frame'].max()}")
    return df


# ---------------------------------------------------------
# 2. BACA LOG JSBSIM (sakti.csv)
# ---------------------------------------------------------
def read_jsbsim_csv(csv_path: str) -> pd.DataFrame:
    print(f"[INFO] Membaca CSV JSBSim: {csv_path}")
    df = pd.read_csv(csv_path)

    print(f"[INFO] Kolom CSV terdeteksi ({len(df.columns)}):")
    for c in df.columns:
        print(f"    {c}")

    # Cari kolom frame counter (nama bisa bervariasi sedikit tergantung
    # versi JSBSim CSV writer, biasanya "sim/sitl-frame-counter" persis
    # atau ada prefix/suffix unit)
    frame_col = None
    for c in df.columns:
        if "sitl-frame-counter" in c:
            frame_col = c
            break
    if frame_col is None:
        print("[ERROR] Kolom frame counter tidak ditemukan di CSV. "
              "Cek nama kolom di atas, sesuaikan manual jika perlu.")
        sys.exit(1)

    df = df.rename(columns={frame_col: "Frame"})
    df["Frame"] = df["Frame"].round().astype("int64")

    print(f"[INFO] {len(df)} baris terbaca dari JSBSim CSV.")
    print(f"[INFO] Range Frame JSBSim : {df['Frame'].min()} - {df['Frame'].max()}")
    return df


# ---------------------------------------------------------
# 3. MERGE BERDASARKAN FRAME COUNTER
# ---------------------------------------------------------
def merge_logs(df_ap: pd.DataFrame, df_jsb: pd.DataFrame) -> pd.DataFrame:
    # CSV JSBSim rate-nya lebih jarang dari ArduPilot log (lihat diskusi:
    # lompatan ~600 per baris CSV vs increment 1 per step ArduPilot).
    # Maka pakai merge_asof (nearest) supaya tiap baris JSBSim CSV
    # dipasangkan ke baris ArduPilot dengan Frame TERDEKAT, bukan exact
    # match (yang akan menghasilkan 0 baris karena granularitas beda).
    df_ap_sorted = df_ap.sort_values("Frame").reset_index(drop=True)
    df_jsb_sorted = df_jsb.sort_values("Frame").reset_index(drop=True)

    merged = pd.merge_asof(
        df_jsb_sorted, df_ap_sorted,
        on="Frame", direction="nearest",
        suffixes=("_jsb", "_ap")
    )

    n_ap, n_jsb, n_merged = len(df_ap), len(df_jsb), len(merged)
    print(f"[INFO] Merge (nearest-match by Frame) selesai: {n_merged} baris "
          f"(dari {n_ap} ArduPilot rows, {n_jsb} JSBSim rows).")

    # Cek kualitas match: seberapa jauh Frame JSBSim vs Frame ArduPilot
    # terdekat -- kalau gap terlalu besar, berarti ada masalah lain
    # (misal salah satu log restart, atau drop data).
    if "Frame_ap" in merged.columns:
        gap = (merged["Frame_ap"] - merged["Frame"]).abs()
    else:
        gap = pd.Series([0])
    print(f"[INFO] Gap Frame terbesar setelah nearest-match: {gap.max()}")

    merged["t"] = merged["time_s_ap"] - merged["time_s_ap"].iloc[0]
    return merged


# ---------------------------------------------------------
# 4. DEFLECTION RATE (deg/s) -- pakai data ArduPilot (lebih rapat)
# ---------------------------------------------------------
def compute_deflection_rate(df: pd.DataFrame) -> pd.DataFrame:
    for s in SURFACES:
        pos_col = f"{s}_deg_ap"
        rate_col = f"{s}_rate_dps"
        if pos_col in df.columns:
            df[rate_col] = np.gradient(df[pos_col].values, df["t"].values)
        else:
            print(f"[WARNING] Kolom '{pos_col}' tidak ada, skip rate utk {s}")
    return df


# ---------------------------------------------------------
# 5. AMBIL TORQUE LANGSUNG DARI JSBSIM (sudah dihitung di sana)
# ---------------------------------------------------------
def extract_torque(df: pd.DataFrame) -> pd.DataFrame:
    name_map_torque = {
        "LO": None, "RO": None, "LI": None, "RI": None
    }
    for c in df.columns:
        for s in SURFACES:
            if f"Hinge_torque_{s}_ftlb" in c:
                name_map_torque[s] = c

    for s in SURFACES:
        col = name_map_torque[s]
        if col is None:
            print(f"[WARNING] Kolom torque utk {s} tidak ditemukan, cek nama kolom CSV.")
            continue
        df[f"{s}_torque_Nm"] = df[col] * FT_LB_TO_NM
    return df


# ---------------------------------------------------------
# 6. WORST-CASE SUMMARY
# ---------------------------------------------------------
def summarize_worst_case(df: pd.DataFrame):
    print("\n" + "=" * 60)
    print("REKAP WORST-CASE UNTUK SPEK SERVO")
    print("=" * 60)

    for s in SURFACES:
        rate_col = f"{s}_rate_dps"
        torque_col = f"{s}_torque_Nm"

        max_rate = df[rate_col].abs().max() if rate_col in df.columns else None
        max_torque = df[torque_col].abs().max() if torque_col in df.columns else None

        worst_row = None
        if torque_col in df.columns and df[torque_col].notna().any():
            idx_worst = df[torque_col].abs().idxmax()
            worst_row = df.loc[idx_worst]

        print(f"\n--- Surface: {s} ---")
        if max_rate is not None:
            print(f"  Max deflection rate observed : {max_rate:.2f} deg/s")
            print(f"  -> Rekomendasi speed servo    : {max_rate * SAFETY_MARGIN:.2f} deg/s "
                  f"(margin {SAFETY_MARGIN}x)")
        if max_torque is not None:
            print(f"  Max hinge torque observed     : {max_torque:.4f} N-m")
            print(f"  -> Rekomendasi torque servo    : {max_torque * SAFETY_MARGIN:.4f} N-m "
                  f"(margin {SAFETY_MARGIN}x)")
            print(f"  -> Setara                      : {max_torque * SAFETY_MARGIN * 10.1972:.2f} kg-cm")
        if worst_row is not None:
            print(f"  Kondisi saat worst-case torque:")
            print(f"    t = {worst_row['t']:.3f} s")
            if "qbar-psf" in [c.split("/")[-1] for c in df.columns]:
                qbar_col = [c for c in df.columns if "qbar-psf" in c][0]
                print(f"    qbar = {worst_row[qbar_col]:.2f} psf")

    print("\n" + "=" * 60)
    print("CATATAN:")
    print("- Cm_hinge yang dipakai JSBSim MASIH ESTIMASI flat-plate kasar,")
    print("  belum divalidasi CFD/wind-tunnel. Margin 2.0x sudah konservatif")
    print("  tapi tetap rekomendasi, bukan keputusan final desain.")
    print("=" * 60 + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bin", required=True)
    parser.add_argument("--csv", required=True)
    parser.add_argument("--out", default="merged_analysis.csv")
    args = parser.parse_args()

    df_ap = read_ardupilot_svod(args.bin)
    df_jsb = read_jsbsim_csv(args.csv)
    merged = merge_logs(df_ap, df_jsb)

    merged = compute_deflection_rate(merged)
    merged = extract_torque(merged)

    merged.to_csv(args.out, index=False)
    print(f"[INFO] Hasil merge + analisa disimpan ke: {args.out}")

    summarize_worst_case(merged)


if __name__ == "__main__":
    main()