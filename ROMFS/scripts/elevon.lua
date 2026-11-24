gcs:send_text(0, "ELEVON MIXER ACTIVE V9 (crow + TECS dive, thr 50)")

-- Output fisik ArduPilot:
--   SERVO1 = ch0 = Left Outboard            (fungsi 0, ditulis skrip ini)
--   SERVO2 = ch1 = Right Outboard           (fungsi 0, ditulis skrip ini)
--   SERVO3 = ch2 = Inboard, kedua sisi      (fungsi 19, dibaca jembatan JSBSim)
--   SERVO4 = ch3 = fungsi 19, TIDAK dibaca jembatan -> referensi elevator
--   SERVO8 = ch7 = perintah crow            (fungsi 0, diset misi DO_SET_SERVO)
local SERVO_OUTBOARD_L_CH = 0
local SERVO_OUTBOARD_R_CH = 1
local SERVO_INBOARD_CH    = 2
local SERVO_ELEV_REF_CH   = 3
local SERVO_CROW_CMD_CH   = 7

-- WAJIB didefinisikan sebelum dipakai. Lua ArduPilot tidak punya
-- constrain global; V3 memanggilnya dari get_crow() sebelum fungsi ini
-- ada, sehingga skrip crash di pemanggilan pertama dan servo outboard
-- berhenti dikendalikan.
local function constrain(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

-- ===================== CROW BRAKE =====================
-- Hambatan dari defleksi permukaan (Drag_surfaces di sakti.xml) memakai
-- |defleksi|, jadi outboard dan inboard yang didefleksikan BERLAWANAN
-- menambah hambatan tanpa memutar pesawat.
--
-- Netral-pitch: Cm_out * d_out + Cm_in * d_in = 0
--   Cm_out = 0.09511, Cm_in = 0.07792 (VLAero, stabil 1.221-1.228
--   sepanjang alpha 5-15 deg) -> d_in = -1.221 * d_out
-- Netral-roll: kedua outboard digeser sama besar.
--
-- ARAH: outboard trailing edge NAIK (+), inboard TURUN (-). Ini
-- mempertahankan lintasan NOSE-UP inboard untuk loop pitch, yang
-- dibutuhkan untuk melawan kelebihan kecepatan saat menukik (run104:
-- 47.3 m/s melewati AIRSPEED_MAX 45). Membalik arah cukup ganti
-- CROW_SIGN menjadi -1.
--
-- Pemicu: DO_SET_SERVO 8 <pwm> di misi.  1500 = mati, 1900 = penuh.
-- Besar: SCR_USER1 = defleksi crow outboard pada perintah penuh (deg).
--        0 atau kosong -> 6.5 deg (inboard 7.9 deg, sisa ~4 deg menunduk).
--        Dibatasi 9.83 deg: di atas itu inboard melewati +-12 deg dan
--        crow tidak lagi netral-pitch.
local OUT_RANGE_DEG  = 18.0
local IN_RANGE_DEG   = 12.0
local CANCEL_RATIO   = 0.09511 / 0.07792
local CROW_DEF_DEG   = 6.5
local CROW_MAX_DEG   = IN_RANGE_DEG / CANCEL_RATIO
local CROW_SIGN      = 1.0

local function crow_out_deg()
    local v = param:get('SCR_USER1')
    if v == nil or v <= 0 then v = CROW_DEF_DEG end
    return constrain(v, 0.0, CROW_MAX_DEG)
end

local function get_crow_frac()
    local pwm = SRV_Channels:get_output_pwm_chan(SERVO_CROW_CMD_CH)
    if pwm == nil then return 0.0 end
    return constrain((pwm - 1500.0) / 400.0, 0.0, 1.0)
end

-- ============== PARAMETER TECS SELAMA CROW ============
-- run108: TECS menaikkan speed demand maksimal 0.5*CLMB_MAX*g/V
-- (AP_TECS.cpp:502) = 0.47 m/s^2, jadi 30->45 m/s butuh ~32 s padahal
-- tukikan hanya 9 s; filter HDEM_TCONST 3 s membuat hdem tertinggal ramp.
-- Selama crow aktif kedua parameter diganti, lalu dikembalikan saat crow
-- mati. CLMB_MAX di TECS hanya dipakai untuk laju speed demand, batas laju
-- NAIK hdem, dan gain/FF throttle (throttle ~0 saat menukik).
-- param:set TIDAK menyimpan ke EEPROM; reboot selalu kembali normal.
local DIVE_PARAMS = {
    { name = 'TECS_CLMB_MAX',    value = 15.0 },  -- speed demand 2.4 m/s^2 @31 m/s
    { name = 'TECS_HDEM_TCONST', value = 1.0  },
    -- run109: throttle penuh -> dorong 100-134 N > hambatan crow, energi naik.
    -- Peta dorong (run105-110): di atas 34 m/s throttle <=60% -> dorong
    -- -4..+7 N. 50% = motor tetap hidup & dikendalikan TECS, dorong ~0.
    -- (JANGAN 0: THR_MAX==0 memicu mode glide TECS, AP_TECS.cpp:1296.)
    { name = 'THR_MAX',          value = 50.0 },
    -- run110: throttle mentok di THR_MAX -> TECS menurunkan _TASmax ke
    -- AIRSPEED_CRUISE (AP_TECS.cpp:408-413) -> spdem tertahan ~35.
    -- Target kecepatan AUTO tetap dari DO_CHANGE_SPEED (navigation.cpp:135).
    { name = 'AIRSPEED_CRUISE',  value = 45.0 },
    -- run110: dpitch jenuh di -20 -> TECS mengecilkan laju turun hdem
    -- (_max_sink_scaler, AP_TECS.cpp:601). -25 = PTCH_LIM_MIN_DEG.
    { name = 'TECS_PITCH_MIN',   value = -25.0 },
}
local dive_saved = nil

-- STATUSTEXT maksimal 50 karakter -> satu baris per parameter.
local function update_dive_params(active)
    if active and dive_saved == nil then
        dive_saved = {}
        for _, p in ipairs(DIVE_PARAMS) do
            local v = param:get(p.name)
            if v ~= nil then
                dive_saved[p.name] = v
                param:set(p.name, p.value)
                gcs:send_text(5, string.format("DIVE ON %s %.1f->%.1f", p.name, v, p.value))
            end
        end
    elseif not active and dive_saved ~= nil then
        for _, p in ipairs(DIVE_PARAMS) do
            local v = dive_saved[p.name]
            if v ~= nil then
                param:set(p.name, v)
                gcs:send_text(5, string.format("DIVE OFF %s ->%.1f", p.name, v))
            end
        end
        dive_saved = nil
    end
end
-- ======================================================


local function pwm_to_norm(pwm)
    if pwm == nil then
        return 0.0
    end
    return constrain((pwm - 1500.0) / 500.0, -1.0, 1.0)
end

local function norm_to_pwm(norm)
    norm = constrain(norm, -1.0, 1.0)
    return math.floor(1500.0 + (norm * 500.0))
end

local function get_elevator_norm()
    -- Dibaca dari SERVO4, bukan SERVO3: SERVO3 ditimpa skrip ini saat
    -- crow aktif, jadi membacanya akan menghasilkan umpan balik ke
    -- keluaran sendiri. SERVO3 dan SERVO4 disetel identik (MIN/MAX/
    -- TRIM/REVERSED/FUNCTION).
    local pwm = SRV_Channels:get_output_pwm_chan(SERVO_ELEV_REF_CH)

    -- Fallback untuk test MANUAL langsung dari RC channel 2
    if pwm == nil then
        pwm = rc:get_pwm(2)
    end

    return pwm_to_norm(pwm)
end

local function get_aileron_norm()
    -- Primary source: ArduPilot aileron output function = 4
    local pwm = SRV_Channels:get_output_pwm(4)

    -- Fallback untuk test MANUAL langsung dari RC channel 1
    if pwm == nil then
        pwm = rc:get_pwm(1)
    end

    return pwm_to_norm(pwm)
end

local dbg_counter = 0

local function update()
    local elev = get_elevator_norm()
    local ail  = get_aileron_norm()

    local P_SCALE = 12.0 / 18.0
    local R_SCALE = 12.0 / 18.0

    local frac    = get_crow_frac()
    update_dive_params(frac > 0.001)
    local d_out   = CROW_SIGN * frac * crow_out_deg()
    local d_in    = -CANCEL_RATIO * d_out
    local c_out   = d_out / OUT_RANGE_DEG
    local c_in    = d_in  / IN_RANGE_DEG

    local LO = constrain((elev * P_SCALE) - (ail * R_SCALE) + c_out, -1.0, 1.0)
    local RO = constrain((elev * P_SCALE) + (ail * R_SCALE) + c_out, -1.0, 1.0)

    SRV_Channels:set_output_pwm_chan_timeout(SERVO_OUTBOARD_L_CH, norm_to_pwm(LO), 100)
    SRV_Channels:set_output_pwm_chan_timeout(SERVO_OUTBOARD_R_CH, norm_to_pwm(RO), 100)

    -- Inboard hanya ditimpa saat crow aktif. Saat crow mati, SERVO3
    -- dibiarkan dikendalikan ArduPlane langsung, sehingga perilakunya
    -- identik dengan V2 yang sudah tervalidasi. Kalau skrip berhenti,
    -- timpaan kedaluwarsa 100 ms dan SERVO3 kembali ke elevator murni.
    if frac > 0.001 then
        local IN = constrain(elev + c_in, -1.0, 1.0)
        SRV_Channels:set_output_pwm_chan_timeout(SERVO_INBOARD_CH, norm_to_pwm(IN), 100)
    end

    dbg_counter = dbg_counter + 1
    if dbg_counter >= 250 then
        dbg_counter = 0
        if frac > 0.001 then
            gcs:send_text(6, string.format("CROW %.0f%% out %+.1f in %+.1f deg",
                frac * 100.0, d_out, d_in))
        end
    end

    return update, 20
end

return update, 20
