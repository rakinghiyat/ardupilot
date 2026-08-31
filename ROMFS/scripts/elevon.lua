gcs:send_text(0, "ELEVON MIXER ACTIVE V2")

-- Output fisik ArduPilot:
-- SERVO1 = channel index 0 = Left Outboard
-- SERVO2 = channel index 1 = Right Outboard
local SERVO_OUTBOARD_L_CH = 0
local SERVO_OUTBOARD_R_CH = 1

local dbg_counter = 0

local last_lo_pwm = 1500
local last_ro_pwm = 1500

local function constrain(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

local function pwm_to_norm(pwm)
    if pwm == nil then
        return 0.0
    end

    local norm = (pwm - 1500.0) / 500.0
    return constrain(norm, -1.0, 1.0)
end

local function norm_to_pwm(norm)
    norm = constrain(norm, -1.0, 1.0)

    local pwm = 1500.0 + (norm * 500.0)

    if pwm > 2000 then pwm = 2000 end
    if pwm < 1000 then pwm = 1000 end

    return math.floor(pwm)
end

local function get_elevator_norm()
    -- Primary source: ArduPilot elevator output function = 19
    local pwm = SRV_Channels:get_output_pwm(19)

    -- Fallback untuk test MANUAL langsung dari RC channel 2
    if pwm == nil then
        pwm = rc:get_pwm(2)
    end

    return pwm_to_norm(pwm)
end

local function get_aileron_norm()
    -- Primary source: ArduPilot aileron output function = 4
    -- Ini yang dibutuhkan untuk AUTO/FBWA
    local pwm = SRV_Channels:get_output_pwm(4)

    -- Fallback untuk test MANUAL langsung dari RC channel 1
    if pwm == nil then
        pwm = rc:get_pwm(1)
    end

    return pwm_to_norm(pwm)
end

local function update()
    local elev = get_elevator_norm()
    local ail  = get_aileron_norm()

    -- Pitch contribution untuk outboard
    local P_SCALE = 12.0 / 18.0

    -- Roll contribution untuk outboard
    -- Untuk test awal boleh tetap 6/18.
    -- Kalau authority kurang, nanti bisa naik ke 9/18 atau 12/18.
    local R_SCALE = 12.0 / 18.0

    local LO = constrain((elev * P_SCALE) - (ail * R_SCALE), -1.0, 1.0)
    local RO = constrain((elev * P_SCALE) + (ail * R_SCALE), -1.0, 1.0)

    local lo_pwm = norm_to_pwm(LO)
    local ro_pwm = norm_to_pwm(RO)

    last_lo_pwm = lo_pwm
    last_ro_pwm = ro_pwm

    SRV_Channels:set_output_pwm_chan_timeout(SERVO_OUTBOARD_L_CH, lo_pwm, 100)
    SRV_Channels:set_output_pwm_chan_timeout(SERVO_OUTBOARD_R_CH, ro_pwm, 100)

    dbg_counter = dbg_counter + 1

    if dbg_counter >= 50 then
        dbg_counter = 0

        -- gcs:send_text(
        --     6,
        --     string.format(
        --         "MIX V2 elev=%.2f ail=%.2f LO=%.2f RO=%.2f PWM_LO=%d PWM_RO=%d",
        --         elev,
        --         ail,
        --         LO,
        --         RO,
        --         last_lo_pwm,
        --         last_ro_pwm
        --     )
        -- )
    end

    return update, 20
end

return update, 20