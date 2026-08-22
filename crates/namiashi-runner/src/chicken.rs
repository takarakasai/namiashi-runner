//! チキンヘッド（胴体が動いても頭を空間に固定して見せる動き）。
//!
//! namiashi の腕は `arm_pitch_joint` の 1 自由度（ピッチ）なので、打ち消せる
//! のは胴体のピッチだけ。IMU のピッチを逆向きに腕へ入れる。
//!
//! ```text
//! q_arm = base − gain · pitch      （一次遅れを通してから出力）
//! ```
//!
//! 一次遅れを入れるのは、IMU の生ピッチをそのままサーボへ流すと、歩行中の
//! 微振動がそのまま腕の唸りになるため。時定数は `poses.chicken_head_tau_s`。

use crate::config::PoseConfig;

/// チキンヘッドの補償器。
#[derive(Debug, Clone)]
pub struct ChickenHead {
    base_rad: f64,
    gain: f64,
    tau_s: f64,
    /// 現在の出力（一次遅れの状態）。
    q: f64,
}

impl ChickenHead {
    pub fn new(cfg: &PoseConfig) -> Self {
        Self {
            base_rad: cfg.chicken_head_base_rad,
            gain: cfg.chicken_head_gain,
            tau_s: cfg.chicken_head_tau_s.max(0.0),
            q: cfg.chicken_head_base_rad,
        }
    }

    /// 1 周期進めて腕の目標角を返す。
    ///
    /// `enabled` が false のときも同じ一次遅れで基準角へ戻す。切った瞬間に
    /// 腕が跳ねないようにするため。
    pub fn update(&mut self, enabled: bool, body_pitch_rad: f64, dt: f64) -> f64 {
        let target = if enabled {
            self.base_rad - self.gain * body_pitch_rad
        } else {
            self.base_rad
        };
        // 一次遅れ: q += (target − q) · dt / (tau + dt)。tau = 0 なら素通し。
        let alpha = if self.tau_s > 0.0 {
            dt / (self.tau_s + dt)
        } else {
            1.0
        };
        self.q += (target - self.q) * alpha.clamp(0.0, 1.0);
        self.q
    }

    /// 基準角へ戻す（脱力時など、追従の履歴を捨てたいとき）。
    pub fn reset(&mut self) {
        self.q = self.base_rad;
    }

    /// 現在の出力。
    #[allow(dead_code)]
    pub fn position(&self) -> f64 {
        self.q
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg(tau: f64, gain: f64, base: f64) -> PoseConfig {
        PoseConfig {
            greeting: "greeting".into(),
            greeting_alt: String::new(),
            chicken_head_base_rad: base,
            chicken_head_gain: gain,
            chicken_head_tau_s: tau,
        }
    }

    #[test]
    fn it_cancels_the_body_pitch_in_steady_state() {
        let mut c = ChickenHead::new(&cfg(0.05, 1.0, 0.0));
        for _ in 0..2000 {
            c.update(true, 0.3, 0.005);
        }
        assert!((c.position() + 0.3).abs() < 1e-6, "{}", c.position());
    }

    #[test]
    fn the_base_angle_is_the_reference_not_zero() {
        let mut c = ChickenHead::new(&cfg(0.0, 1.0, -0.4));
        assert_eq!(c.update(true, 0.0, 0.005), -0.4);
        assert!((c.update(true, 0.2, 0.005) + 0.6).abs() < 1e-12);
    }

    #[test]
    fn a_zero_time_constant_passes_straight_through() {
        let mut c = ChickenHead::new(&cfg(0.0, 1.0, 0.0));
        assert!((c.update(true, 0.25, 0.005) + 0.25).abs() < 1e-12);
    }

    #[test]
    fn the_filter_slows_the_response_down() {
        let mut fast = ChickenHead::new(&cfg(0.0, 1.0, 0.0));
        let mut slow = ChickenHead::new(&cfg(0.5, 1.0, 0.0));
        let f = fast.update(true, 1.0, 0.005);
        let s = slow.update(true, 1.0, 0.005);
        assert!(s.abs() < f.abs() * 0.1, "fast={f} slow={s}");
    }

    #[test]
    fn turning_it_off_eases_back_to_the_base_angle() {
        let mut c = ChickenHead::new(&cfg(0.05, 1.0, 0.0));
        for _ in 0..2000 {
            c.update(true, 0.3, 0.005);
        }
        let first_off = c.update(false, 0.3, 0.005);
        // 一気に 0 へは戻らない（跳ねない）。
        assert!(first_off < -0.2, "{first_off}");
        for _ in 0..2000 {
            c.update(false, 0.3, 0.005);
        }
        assert!(c.position().abs() < 1e-6);
    }

    #[test]
    fn reset_returns_to_the_base_angle_immediately() {
        let mut c = ChickenHead::new(&cfg(0.05, 1.0, -0.2));
        for _ in 0..100 {
            c.update(true, 0.5, 0.005);
        }
        c.reset();
        assert_eq!(c.position(), -0.2);
    }
}
