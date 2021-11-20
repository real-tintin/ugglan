from collections import deque


class RollingSimData:
    def __init__(self, n_samples: int):
        self.t_s = deque(maxlen=n_samples)

        self.roll_ang = deque(maxlen=n_samples)
        self.roll_rate = deque(maxlen=n_samples)
        self.roll_acc = deque(maxlen=n_samples)

        self.pitch_ang = deque(maxlen=n_samples)
        self.pitch_rate = deque(maxlen=n_samples)
        self.pitch_acc = deque(maxlen=n_samples)

        self.yaw_ang = deque(maxlen=n_samples)
        self.yaw_rate = deque(maxlen=n_samples)
        self.yaw_acc = deque(maxlen=n_samples)

        self.est_roll_ang = deque(maxlen=n_samples)
        self.est_pitch_ang = deque(maxlen=n_samples)
        self.est_yaw_ang = deque(maxlen=n_samples)

        self.est_roll_rate = deque(maxlen=n_samples)
        self.est_pitch_rate = deque(maxlen=n_samples)
        self.est_yaw_rate = deque(maxlen=n_samples)

        self.est_roll_acc = deque(maxlen=n_samples)
        self.est_pitch_acc = deque(maxlen=n_samples)
        self.est_yaw_acc = deque(maxlen=n_samples)

        self.ref_fz = deque(maxlen=n_samples)
        self.ref_roll = deque(maxlen=n_samples)
        self.ref_pitch = deque(maxlen=n_samples)
        self.ref_yaw_rate = deque(maxlen=n_samples)

        self.ctrl_fz = deque(maxlen=n_samples)
        self.ctrl_mx = deque(maxlen=n_samples)
        self.ctrl_my = deque(maxlen=n_samples)
        self.ctrl_mz = deque(maxlen=n_samples)

    def update(self,
               t_s: float,

               roll_ang: float,
               roll_rate: float,
               roll_acc: float,

               pitch_ang: float,
               pitch_rate: float,
               pitch_acc: float,

               yaw_ang: float,
               yaw_rate: float,
               yaw_acc: float,

               est_roll_ang: float,
               est_pitch_ang: float,
               est_yaw_ang: float,

               est_roll_rate: float,
               est_pitch_rate: float,
               est_yaw_rate: float,

               est_roll_acc: float,
               est_pitch_acc: float,
               est_yaw_acc: float,

               ref_fz: float,
               ref_roll: float,
               ref_pitch: float,
               ref_yaw_rate: float,

               ctrl_fz: float,
               ctrl_mx: float,
               ctrl_my: float,
               ctrl_mz: float,
               ):
        self.t_s.append(t_s)

        self.roll_ang.append(roll_ang)
        self.roll_rate.append(roll_rate)
        self.roll_acc.append(roll_acc)

        self.pitch_ang.append(pitch_ang)
        self.pitch_rate.append(pitch_rate)
        self.pitch_acc.append(pitch_acc)

        self.yaw_ang.append(yaw_ang)
        self.yaw_rate.append(yaw_rate)
        self.yaw_acc.append(yaw_acc)

        self.est_roll_ang.append(est_roll_ang)
        self.est_pitch_ang.append(est_pitch_ang)
        self.est_yaw_ang.append(est_yaw_ang)

        self.est_roll_rate.append(est_roll_rate)
        self.est_pitch_rate.append(est_pitch_rate)
        self.est_yaw_rate.append(est_yaw_rate)

        self.est_roll_acc.append(est_roll_acc)
        self.est_pitch_acc.append(est_pitch_acc)
        self.est_yaw_acc.append(est_yaw_acc)

        self.ref_fz.append(ref_fz)
        self.ref_roll.append(ref_roll)
        self.ref_pitch.append(ref_pitch)
        self.ref_yaw_rate.append(ref_yaw_rate)

        self.ctrl_fz.append(ctrl_fz)
        self.ctrl_mx.append(ctrl_mx)
        self.ctrl_my.append(ctrl_my)
        self.ctrl_mz.append(ctrl_mz)
