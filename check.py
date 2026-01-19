pid_params = {
            'x': {'kp': 4.7, 'ki': 0.0, 'kd': 1.7, 'max_out': self.max_vel},
            'y': {'kp': 4.7, 'ki': 0.0, 'kd': 1.7, 'max_out': self.max_vel},
            'theta': {'kp': 100.0, 'ki': 0.1, 'kd': 5.5, 'max_out': self.max_vel * 2}
}
# print(**pid_params['x'])
print(pid_params['x'])