#ifndef ROBOT_EMOTES_H
#define ROBOT_EMOTES_H



// -------- Forward declarations of Gait_Mechanism exports we call --------
bool solve_leg_ik_3dof(float tx, float ty, float tz, float urdf_x_offset,
                       float &hip, float &thigh, float &knee);
void setServo(int id, float angle);
extern const char* LEG_ORDER[6];

// ============================================================
// EMOTE-MODE CONSTANTS (safe to tune)
// ============================================================
// Reference body height. Matches BODY_HEIGHTS in the Python sim so the emote poses look identical to what you saw in PyBullet.
static const float EMOTE_BODY_HEIGHT = 0.23f;

// from the walking urdf_x_filtered so the look isn't perturbed by whatever direction the robot was last walking in when emote mode was entered.
static const float EMOTE_URDF_X = 0.005f;


// PLAY triggers are ignored until we're within this window so the first frame of the emote doesn't jerk the robot from a mid-ramp height.
static const float EMOTE_HEIGHT_SETTLED_M = 0.003f;

// Leg-index constants matching LEG_ORDER = {"FL","ML","RL","FR","MR","RR"}
enum EmoteLeg { EL_FL = 0, EL_ML = 1, EL_RL = 2, EL_FR = 3, EL_MR = 4, EL_RR = 5 };

// ============================================================
// PER-TICK OUTPUT BUFFER
// ============================================================
struct EmoteTargets {
    float hip[6];
    float thigh[6];
    float knee[6];
    bool  valid[6];
};

// ============================================================
// EMOTE-AUTHORING HELPERS
// ============================================================

static inline void emoteFillDefault(EmoteTargets &t) {
    float h, th, kn;
    bool ok = solve_leg_ik_3dof(0.0f, 0.0f, current_body_height, EMOTE_URDF_X, h, th, kn);
    for (int i = 0; i < 6; i++) {
        t.hip[i] = h; t.thigh[i] = th; t.knee[i] = kn; t.valid[i] = ok;
    }
}

static inline void emoteIK(EmoteTargets &t, int i, float x, float y, float z) {
    float h, th, kn;
    bool ok = solve_leg_ik_3dof(x, y, z, EMOTE_URDF_X, h, th, kn);
    t.hip[i] = h; t.thigh[i] = th; t.knee[i] = kn; t.valid[i] = ok;
}

static inline void emoteDirect(EmoteTargets &t, int i, float h, float th, float kn) {
    t.hip[i] = h; t.thigh[i] = th; t.knee[i] = kn; t.valid[i] = true;
}

static inline float emoteEase(float a) {
    if (a < 0.0f) a = 0.0f; else if (a > 1.0f) a = 1.0f;
    return a * a * (3.0f - 2.0f * a);
}


static inline void emoteLerpFrames(EmoteTargets &out, const EmoteTargets &a,
                                   const EmoteTargets &b, float mix) {
    if (mix < 0.0f) mix = 0.0f; else if (mix > 1.0f) mix = 1.0f;
    for (int i = 0; i < 6; i++) {
        if (a.valid[i] && b.valid[i]) {
            out.hip[i]   = a.hip[i]   * (1.0f - mix) + b.hip[i]   * mix;
            out.thigh[i] = a.thigh[i] * (1.0f - mix) + b.thigh[i] * mix;
            out.knee[i]  = a.knee[i]  * (1.0f - mix) + b.knee[i]  * mix;
            out.valid[i] = true;
        } else if (b.valid[i]) {
            out.hip[i] = b.hip[i]; out.thigh[i] = b.thigh[i];
            out.knee[i] = b.knee[i]; out.valid[i] = true;
        } else {
            out.hip[i] = a.hip[i]; out.thigh[i] = a.thigh[i];
            out.knee[i] = a.knee[i]; out.valid[i] = a.valid[i];
        }
    }
}

// ============================================================
// SERVO WRITER (SHARES MIRRORING WITH THE WALKING CODE)
// ============================================================
static void writeLegServosFromEmote(int idx, float hip, float thigh, float knee) {
    String legName = LEG_ORDER[idx];
    bool is_left  = legName.endsWith("L");
    bool is_right = legName.endsWith("R");

    float actual_th = is_left ? -thigh : thigh;
    float actual_kn = is_left ? -knee  : knee;
    if (is_right) actual_th = -thigh;

    float i_kn = is_left ? -KNEE_ASSEMBLY_OFFSET : KNEE_ASSEMBLY_OFFSET;

    float h_phys_deg  = hip       * 180.0f / PI;
    float th_phys_deg = actual_th * 180.0f / PI;
    float kn_phys_deg = (actual_kn - i_kn) * 180.0f / PI;

    int hipHWID, thighHWID, kneeHWID;
    if      (legName == "FR") { kneeHWID = 0;  thighHWID = 1;  hipHWID = 2;  }
    else if (legName == "FL") { kneeHWID = 3;  thighHWID = 4;  hipHWID = 5;  }
    else if (legName == "MR") { kneeHWID = 6;  thighHWID = 7;  hipHWID = 8;  }
    else if (legName == "ML") { kneeHWID = 9;  thighHWID = 10; hipHWID = 11; }
    else if (legName == "RR") { kneeHWID = 12; thighHWID = 13; hipHWID = 14; }
    else if (legName == "RL") { kneeHWID = 15; thighHWID = 16; hipHWID = 17; }
    else return;

    if (is_left) {
        setServo(hipHWID,   90.0f - h_phys_deg);
        setServo(thighHWID, 90.0f - th_phys_deg);
        setServo(kneeHWID,  90.0f + kn_phys_deg);
    } else {
        setServo(hipHWID,   90.0f - h_phys_deg);
        setServo(thighHWID, 90.0f + th_phys_deg);
        setServo(kneeHWID,  90.0f + kn_phys_deg);
    }
}

// ============================================================
// EMOTE FUNCTIONS  (direct ports of the PyBullet script)
// ============================================================


// ---- Wiggle: gentle side-to-side sway of the whole body ----
static void emote_wiggle(float t, EmoteTargets &out) {
    float y_shift = 0.04f * sin(2.0f * PI * 0.5f * t);
    for (int i = 0; i < 6; i++) emoteIK(out, i, 0.0f, -y_shift, EMOTE_BODY_HEIGHT);
}


static void emote_intimidate(float t, EmoteTargets &out) {
    const float crouch_dur = 3.0f;
    const float trans_dur  = 1.5f;
    const float MID_HIP_LIFT   = 1.5f;
    const float MID_OSC_FREQ   = 1.0f;
    const float MID_OSC_AMP    = 0.6f;
    const float MID_THIGH_BIAS = 0.5f;

    float h_cr, th_cr, kn_cr;
    solve_leg_ik_3dof(0, 0, EMOTE_BODY_HEIGHT * 0.5f,  EMOTE_URDF_X, h_cr, th_cr, kn_cr);
    float h_st, th_st, kn_st;
    solve_leg_ik_3dof(0, 0, EMOTE_BODY_HEIGHT + 0.02f, EMOTE_URDF_X, h_st, th_st, kn_st);
    float h_mid, th_mid, kn_mid;
    solve_leg_ik_3dof(0, 0, EMOTE_BODY_HEIGHT - 0.04f, EMOTE_URDF_X, h_mid, th_mid, kn_mid);

    float bp_h[6], bp_th[6], bp_kn[6];
    bp_h[EL_FL] = 0.5f;  bp_th[EL_FL] = 0.8f;  bp_kn[EL_FL] = -0.2f;
    bp_h[EL_FR] = -0.5f; bp_th[EL_FR] = 0.8f;  bp_kn[EL_FR] = -0.2f;
    bp_h[EL_ML] =  MID_HIP_LIFT; bp_th[EL_ML] = th_mid + MID_THIGH_BIAS; bp_kn[EL_ML] = kn_mid;
    bp_h[EL_MR] = -MID_HIP_LIFT; bp_th[EL_MR] = th_mid + MID_THIGH_BIAS; bp_kn[EL_MR] = kn_mid;
    bp_h[EL_RL] = h_st; bp_th[EL_RL] = th_st; bp_kn[EL_RL] = kn_st;
    bp_h[EL_RR] = h_st; bp_th[EL_RR] = th_st; bp_kn[EL_RR] = kn_st;

    if (t < crouch_dur) {
        for (int i = 0; i < 6; i++) emoteDirect(out, i, h_cr, th_cr, kn_cr);
    } else if (t < crouch_dur + trans_dur) {
        float sa = emoteEase((t - crouch_dur) / trans_dur);
        for (int i = 0; i < 6; i++) {
            emoteDirect(out, i,
                h_cr  + (bp_h[i]  - h_cr)  * sa,
                th_cr + (bp_th[i] - th_cr) * sa,
                kn_cr + (bp_kn[i] - kn_cr) * sa);
        }
    } else {
        float t_local = t - crouch_dur - trans_dur;
        for (int i = 0; i < 6; i++) emoteDirect(out, i, bp_h[i], bp_th[i], bp_kn[i]);
        float th_osc_L = th_mid + MID_THIGH_BIAS + MID_OSC_AMP * sin(2 * PI * MID_OSC_FREQ * t_local + 0.0f);
        float th_osc_R = th_mid + MID_THIGH_BIAS + MID_OSC_AMP * sin(2 * PI * MID_OSC_FREQ * t_local + PI);
        emoteDirect(out, EL_ML,  MID_HIP_LIFT, th_osc_L, kn_mid);
        emoteDirect(out, EL_MR, -MID_HIP_LIFT, th_osc_R, kn_mid);
    }
}

// ---- Victory: sit back, both front paws wave ----

static void emote_victory(float t, EmoteTargets &out) {
    const float sit_dur      = 2.0f;
    const float paw_fade_dur = 1.0f;

    float current_height_factor;
    if (t < sit_dur) {
        float sa = emoteEase(t / sit_dur);
        current_height_factor = 1.0f - 0.3f * sa;
    } else {
        current_height_factor = 0.7f;
    }
    float h_back, th_back, kn_back;
    solve_leg_ik_3dof(0, 0, EMOTE_BODY_HEIGHT * current_height_factor, EMOTE_URDF_X, h_back, th_back, kn_back);
    emoteDirect(out, EL_RL, h_back, th_back, kn_back);
    emoteDirect(out, EL_RR, h_back, th_back, kn_back);

    if (t >= sit_dur) {
        float t_wave = t - sit_dur;
        float paw_alpha = emoteEase(t_wave / paw_fade_dur);

        float wave_L = 0.5f * sin(2 * PI * 0.75f * t_wave);
        float wave_R = 0.5f * cos(2 * PI * 0.75f * t_wave);


        float def_h, def_th, def_kn;
        solve_leg_ik_3dof(0.0f, 0.0f, EMOTE_BODY_HEIGHT, EMOTE_URDF_X, def_h, def_th, def_kn);

        float fl_h  = def_h  * (1.0f - paw_alpha) +  0.6f            * paw_alpha;
        float fl_th = def_th * (1.0f - paw_alpha) + (0.6f + wave_L)  * paw_alpha;
        float fl_kn = def_kn * (1.0f - paw_alpha) +  1.4f            * paw_alpha;
        float fr_h  = def_h  * (1.0f - paw_alpha) + (-0.6f)          * paw_alpha;
        float fr_th = def_th * (1.0f - paw_alpha) + (0.6f + wave_R)  * paw_alpha;
        float fr_kn = def_kn * (1.0f - paw_alpha) +  1.4f            * paw_alpha;

        emoteDirect(out, EL_FL, fl_h, fl_th, fl_kn);
        emoteDirect(out, EL_FR, fr_h, fr_th, fr_kn);
    }
}

// ---- Breathing: gentle chest rise/fall + leg spread ----
static void emote_breathing(float t, EmoteTargets &out) {
    float breath_factor = (sin(2 * PI * 0.3f * t) + 1.0f) * 0.5f;
    float z_height = EMOTE_BODY_HEIGHT - 0.02f + 0.03f * breath_factor;
    float y_spread = 0.015f * breath_factor;
    for (int i = 0; i < 6; i++) {
        String legName = LEG_ORDER[i];
        float side_sign = legName.endsWith("L") ? -1.0f : 1.0f;
        emoteIK(out, i, 0.0f, y_spread * side_sign, z_height);
    }
}

// ---- Stomp: five legs planted, FL taps the ground ----

static void emote_stomp(float t, EmoteTargets &out) {
    for (int i = 0; i < 6; i++) {
        if (i != EL_FL) emoteIK(out, i, -0.01f, 0.01f, EMOTE_BODY_HEIGHT);
    }
    float stomp_cycle = sin(2 * PI * 1.5f * t);
    if (stomp_cycle > 0.0f) {
        emoteIK(out, EL_FL, 0.01f, 0.0f, EMOTE_BODY_HEIGHT - 0.04f);
    } else {
        emoteIK(out, EL_FL, 0.01f, 0.0f, EMOTE_BODY_HEIGHT + 0.005f);
    }
}

// ---- Stitched: breathing 5s -> foot stomp ----
static void emote_breathe_stomp(float t, EmoteTargets &out) {
    const float breathe_dur = 5.0f;
    if (t < breathe_dur) emote_breathing(t, out);
    else                 emote_stomp(t - breathe_dur, out);
}

// ---- Matrix Roll: circular sway of the whole body ----
static void emote_matrix_roll(float t, EmoteTargets &out) {
    const float radius = 0.03f;
    float x_shift = radius * cos(2 * PI * 0.5f * t);
    float y_shift = radius * sin(2 * PI * 0.5f * t);
    for (int i = 0; i < 6; i++) emoteIK(out, i, -x_shift, -y_shift, EMOTE_BODY_HEIGHT);
}

// ---- Shy Peek: front paws cover face and oscillate; rear+middle plant firmly

static void emote_shy_peek(float t, EmoteTargets &out) {
    const float body_z    = EMOTE_BODY_HEIGHT * 0.92f;
    const float rear_shift = 0.030f;
    const float mid_shift  = 0.015f;
    const float rear_splay = 0.020f;
    float sway = 0.010f * sin(2 * PI * 0.5f * t);
    float thigh_osc = 0.85f + 0.15f * sin(2 * PI * 1.0f * t);

    emoteIK(out, EL_ML, mid_shift, sway, body_z);
    emoteIK(out, EL_MR, mid_shift, sway, body_z);
    emoteIK(out, EL_RL, rear_shift, sway - rear_splay, body_z);
    emoteIK(out, EL_RR, rear_shift, sway + rear_splay, body_z);
    emoteDirect(out, EL_FL,  0.7f, thigh_osc, 1.2f);
    emoteDirect(out, EL_FR, -0.7f, thigh_osc, 1.2f);
}

// ---- Sneak: low body, tripod wave-shuffle ----
static void emote_sneak(float t, EmoteTargets &out) {
    float low_height = EMOTE_BODY_HEIGHT * 0.65f;
    float wave_x = 0.03f * sin(2 * PI * 1.0f * t);
    emoteIK(out, EL_FL,  wave_x, 0.0f, low_height);
    emoteIK(out, EL_MR,  wave_x, 0.0f, low_height);
    emoteIK(out, EL_RL,  wave_x, 0.0f, low_height);
    emoteIK(out, EL_FR, -wave_x, 0.0f, low_height);
    emoteIK(out, EL_ML, -wave_x, 0.0f, low_height);
    emoteIK(out, EL_RR, -wave_x, 0.0f, low_height);
}

// ---- Happy Dance: alternating tripod bounce ----
static void emote_happy_dance(float t, EmoteTargets &out) {
    const float freq = 1.25f;
    float phase = sin(2 * PI * freq * t);
    float bounce = 0.02f * fabs(phase);
    float side_tilt = 0.02f * phase;
    float hA = (phase > 0.0f) ? bounce : 0.0f;
    float hB = (phase > 0.0f) ? 0.0f   : bounce;
    emoteIK(out, EL_FL, 0.0f, side_tilt, EMOTE_BODY_HEIGHT - hA);
    emoteIK(out, EL_MR, 0.0f, side_tilt, EMOTE_BODY_HEIGHT - hA);
    emoteIK(out, EL_RL, 0.0f, side_tilt, EMOTE_BODY_HEIGHT - hA);
    emoteIK(out, EL_FR, 0.0f, side_tilt, EMOTE_BODY_HEIGHT - hB);
    emoteIK(out, EL_ML, 0.0f, side_tilt, EMOTE_BODY_HEIGHT - hB);
    emoteIK(out, EL_RR, 0.0f, side_tilt, EMOTE_BODY_HEIGHT - hB);
}

// ---- Cautious Tap: leaned-back stance, right-front paw pokes cautiously

static void emote_cautious_tap(float t, EmoteTargets &out) {
    float lean_height = EMOTE_BODY_HEIGHT * 0.85f;
    emoteIK(out, EL_FL, -0.02f, 0.0f, lean_height);
    emoteIK(out, EL_ML, -0.02f, 0.0f, lean_height);
    emoteIK(out, EL_MR, -0.02f, 0.0f, lean_height);
    emoteIK(out, EL_RL, -0.02f, 0.0f, lean_height);
    emoteIK(out, EL_RR, -0.02f, 0.0f, lean_height);


    float poke_phase = 0.5f + 0.5f * sin(2 * PI * 0.4f * t);

    float pln_h, pln_th, pln_kn;
    solve_leg_ik_3dof(0.06f, -0.02f, lean_height + 0.02f, EMOTE_URDF_X, pln_h, pln_th, pln_kn);
    const float raised_h  = -0.2f;
    const float raised_th =  0.4f;
    const float raised_kn =  0.8f;

    float fr_h  = pln_h  * (1.0f - poke_phase) + raised_h  * poke_phase;
    float fr_th = pln_th * (1.0f - poke_phase) + raised_th * poke_phase;
    float fr_kn = pln_kn * (1.0f - poke_phase) + raised_kn * poke_phase;

    emoteDirect(out, EL_FR, fr_h, fr_th, fr_kn);
}

// ---- Stitched: happy dance -> sneak, with a proper 2-second crossfade

static void emote_dance_sneak(float t, EmoteTargets &out) {
    const float dance_dur = 4.0f;
    const float fade_half = 1.0f;  // total crossfade window = 2 * fade_half

    if (t < dance_dur - fade_half) {
        emote_happy_dance(t, out);
    } else if (t < dance_dur + fade_half) {
        EmoteTargets dance_pose, sneak_pose;
        emoteFillDefault(dance_pose);
        emoteFillDefault(sneak_pose);
        emote_happy_dance(t, dance_pose);

        float sneak_t = t - (dance_dur - fade_half);
        emote_sneak(sneak_t, sneak_pose);
        float mix = emoteEase((t - (dance_dur - fade_half)) / (2.0f * fade_half));
        emoteLerpFrames(out, dance_pose, sneak_pose, mix);
    } else {
        emote_sneak(t - (dance_dur - fade_half), out);
    }
}

// ---- Head Tilt: curious puppy sway with a tiny forward lean ----
static void emote_head_tilt(float t, EmoteTargets &out) {
    float roll = sin(2 * PI * 0.4f * t);
    float lean = 0.01f * (sin(2 * PI * 0.2f * t) * 0.5f + 0.5f);
    for (int i = 0; i < 6; i++) {
        String legName = LEG_ORDER[i];
        bool is_left = legName.endsWith("L");
        float dz = 0.03f * roll * (is_left ? 1.0f : -1.0f);
        emoteIK(out, i, lean, 0.0f, EMOTE_BODY_HEIGHT + dz);
    }
}

// ---- Play Bow: front sinks low, rear stays tall, small butt wiggle ----

static void emote_play_bow(float t, EmoteTargets &out) {
    const float down_dur = 1.5f;
    float a = (t < down_dur) ? emoteEase(t / down_dur) : 1.0f;
    float front_z = EMOTE_BODY_HEIGHT - 0.09f * a;
    float rear_z  = EMOTE_BODY_HEIGHT + 0.015f * a;
    float wiggle  = (t >= down_dur) ? 0.02f * sin(2 * PI * 1.5f * t) : 0.0f;

    emoteIK(out, EL_FL, 0.02f * a, 0.0f, front_z);
    emoteIK(out, EL_FR, 0.02f * a, 0.0f, front_z);
    emoteIK(out, EL_ML, 0.0f, 0.0f, EMOTE_BODY_HEIGHT - 0.03f * a);
    emoteIK(out, EL_MR, 0.0f, 0.0f, EMOTE_BODY_HEIGHT - 0.03f * a);
    emoteIK(out, EL_RL, -0.01f, wiggle * -1.0f, rear_z);
    emoteIK(out, EL_RR, -0.01f, wiggle *  1.0f, rear_z);
}

// ---- Pushups: front + middle bend rhythmically, rear planted ----
static void emote_pushups(float t, EmoteTargets &out) {
    float cycle = (sin(2 * PI * 0.6f * t - PI * 0.5f) + 1.0f) * 0.5f;
    emoteIK(out, EL_FL, 0.0f, 0.0f, EMOTE_BODY_HEIGHT - 0.07f  * cycle);
    emoteIK(out, EL_FR, 0.0f, 0.0f, EMOTE_BODY_HEIGHT - 0.07f  * cycle);
    emoteIK(out, EL_ML, 0.0f, 0.0f, EMOTE_BODY_HEIGHT - 0.035f * cycle);
    emoteIK(out, EL_MR, 0.0f, 0.0f, EMOTE_BODY_HEIGHT - 0.035f * cycle);
}

// ---- Wave Hello: settle onto rear+middle, right-front waves ----

static void emote_wave_hello(float t, EmoteTargets &out) {
    const float sit_dur = 1.5f;
    float a = emoteEase(t / sit_dur);
    float back_z = EMOTE_BODY_HEIGHT * (1.0f - 0.25f * a);
    emoteIK(out, EL_RL, 0.0f, 0.0f, back_z);
    emoteIK(out, EL_RR, 0.0f, 0.0f, back_z);
    emoteIK(out, EL_ML, -0.01f, 0.0f, EMOTE_BODY_HEIGHT * (1.0f - 0.1f * a));
    emoteIK(out, EL_MR, -0.01f, 0.0f, EMOTE_BODY_HEIGHT * (1.0f - 0.1f * a));
    emoteIK(out, EL_FL,  0.01f, 0.0f, EMOTE_BODY_HEIGHT);

    float def_h, def_th, def_kn;
    solve_leg_ik_3dof(0.01f, 0.0f, EMOTE_BODY_HEIGHT, EMOTE_URDF_X, def_h, def_th, def_kn);
    float wave = 0.45f * sin(2 * PI * 1.2f * t);
    float fr_h  = def_h  * (1.0f - a) + (-0.5f)         * a;
    float fr_th = def_th * (1.0f - a) + (0.7f + wave)   * a;
    float fr_kn = def_kn * (1.0f - a) +  1.3f           * a;
    emoteDirect(out, EL_FR, fr_h, fr_th, fr_kn);
}

// ---- Stadium Ripple: each foot lifts in sequence ----
static void emote_ripple(float t, EmoteTargets &out) {
    const int   order[6] = { EL_FL, EL_ML, EL_RL, EL_RR, EL_MR, EL_FR };
    const float period   = 0.35f;
    const float cycle_len = 6 * period;
    float tc = fmod(t, cycle_len);
    for (int i = 0; i < 6; i++) {
        float start = i * period;
        float lift = 0.0f;
        if (tc >= start && tc < start + period) {
            float ph = (tc - start) / period;
            lift = 0.05f * sin(PI * ph);
        }
        emoteIK(out, order[i], 0.0f, 0.0f, EMOTE_BODY_HEIGHT - lift);
    }
}

// ---- Itch Scratch: weight on 5 legs, right-rear scratches rapidly ----

static void emote_scratch(float t, EmoteTargets &out) {
    for (int i = 0; i < 6; i++) {
        if (i != EL_RR) emoteIK(out, i, 0.0f, -0.015f, EMOTE_BODY_HEIGHT * 0.92f);
    }
    float scr = 0.5f * sin(2 * PI * 2.0f * t);
    emoteDirect(out, EL_RR, -0.8f, 0.2f + scr, 1.6f + 0.3f * scr);
}

// ============================================================
// EMOTE CATALOG  (single source of truth for order + names + timing)
// ============================================================

typedef void (*EmoteFn)(float t, EmoteTargets &out);
struct EmoteEntry {
    const char* name;
    float       duration_sec;
    float       intro_sec;
    float       outro_sec;
    EmoteFn     fn;
};

static const EmoteEntry EMOTES[] = {
    //  name                          dur   intro  outro  fn
    { "Curious Head Tilt",           10.0f,  0.0f,  0.0f, emote_head_tilt     },
    { "Shy Peek-a-boo",               6.0f,  1.0f,  1.0f, emote_shy_peek      },
    { "Cautious Object Tap",          7.0f,  1.0f,  1.0f, emote_cautious_tap  },
    { "The Wiggle",                   9.0f,  0.0f,  0.0f, emote_wiggle        },
    { "Play Bow",                     6.0f,  0.0f,  1.0f, emote_play_bow      },
    { "Happy Dance into Sneak",      10.0f,  0.0f,  1.5f, emote_dance_sneak   },
    { "Stadium Ripple Wave",         10.0f,  0.0f,  0.0f, emote_ripple        },
    { "Breathing into Foot Stomp",    9.0f,  0.0f,  0.0f, emote_breathe_stomp },
    { "Itch Scratch",                 7.0f,  1.0f,  1.0f, emote_scratch       },
    { "Matrix Gyro Roll",             5.0f,  0.5f,  0.5f, emote_matrix_roll   },
    { "Push-Ups",                    12.0f,  0.0f,  1.5f, emote_pushups       },
    { "Sit & Wave Hello",             8.0f,  0.0f,  1.0f, emote_wave_hello    },
    { "Victory Wave",                 9.0f,  0.0f,  1.0f, emote_victory       },
    { "Battle Mode / Intimidate",     9.0f,  1.0f,  1.0f, emote_intimidate    }
};
static const uint8_t NUM_EMOTES = sizeof(EMOTES) / sizeof(EMOTES[0]);
static const uint8_t EMOTE_NONE = 255;

// ============================================================
// PER-TICK ENTRY POINT (called from KinematicsTask)
// ============================================================

void runEmoteTick() {
    EmoteTargets targets;
    emoteFillDefault(targets);

    if (emote_playing && emote_playing_id < NUM_EMOTES) {
        float t     = (millis() - emote_start_ms) / 1000.0f;
        float dur   = EMOTES[emote_playing_id].duration_sec;
        float intro = EMOTES[emote_playing_id].intro_sec;
        float outro = EMOTES[emote_playing_id].outro_sec;

        if (t >= dur) {
            emote_playing    = false;
            emote_playing_id = EMOTE_NONE;
            // targets stays at default -- clean landing at standing pose.
        } else {
            float alpha = 1.0f;
            if (intro > 0.0f && t < intro) {
                alpha = emoteEase(t / intro);
            } else if (outro > 0.0f && t > (dur - outro)) {
                alpha = emoteEase((dur - t) / outro);
            }

            EmoteTargets emote_pose;
            emoteFillDefault(emote_pose);
            EMOTES[emote_playing_id].fn(t, emote_pose);

            if (alpha >= 0.999f) {
                targets = emote_pose;
            } else {
                emoteLerpFrames(targets, targets /*default*/, emote_pose, alpha);
            }
        }
    }

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        for (int i = 0; i < 6; i++) {
            if (targets.valid[i]) {
                writeLegServosFromEmote(i, targets.hip[i], targets.thigh[i], targets.knee[i]);
            }
        }
        xSemaphoreGive(i2cMutex);
    }
}

#endif // ROBOT_EMOTES_H
