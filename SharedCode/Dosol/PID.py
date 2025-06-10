class PIDController:
    def __init__(self, kp, ki, kd, set_point):
        self.kp = kp  # 비례 상수
        self.ki = ki  # 적분 상수
        self.kd = kd  # 미분 상수
        self.set_point = set_point  # 목표 값
        self.last_error = 0
        self.integral = 0

    def update(self, current_value, dt):
        """
        PID 컨트롤러 업데이트 메서드
        :param current_value: 현재 값
        :param dt: 시간 간격
        :return: 조정된 제어 입력
        """
        error = self.set_point - current_value
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error
        return output
    

"""
# pidcal.py
# 최종 수정일자 : 2024-05-10
# 작업자 : 박도솔 

#######################################################################

class PidCal:
    error_sum = 0
    error_old = 0
    p = [0.3, 0.0005, 0.03] # optimized kp,ki,kd
    #p = [0.0020, 0.000005, 0.005]
    dp = [p[0]/10, p[1]/10, p[2]/10] # to twiddle kp, ki, kd

    def __init__(self): # 초기화
        # print "init PidCal"
        self.x = 0

    def cal_error(self, setpoint=0): # 오차 계산
        return setpoint - self.x

    def info_p(self): # P게인 값 반환(반올림 7자리)
        round_p= [round(x,7) for x in self.p]
        return round_p

    #########################################################################
    '''
    By GPT
    twiddle 알고리즘은 PID 게인값을 최적화하는 알고리즘입니다.

    1. 초기 오차값을 계산합니다.
    2. 설정된 임계값(threshold)보다 게인값이 큰 동안 반복합니다.
    3. 각각의 게인값에 대해 새로운 값으로 이동하여 오차를 계산합니다.
    4. 새로운 값이 더 나은 결과를 가져온 경우, 해당 게인값을 업데이트하고, 게인값의 조정폭(dp)을 증가시킵니다.
    5. 그렇지 않은 경우, 해당 게인값을 반대 방향으로 움직여 다시 시도하고, 결과가 개선되면 해당 게인값을 업데이트하고, dp를 약간 증가시킵니다.
    6. 아무런 개선이 없는 경우, 게인값을 원래 위치로 되돌리고 dp를 축소합니다.
    7. 설정된 임계값에 도달할 때까지 이 프로세스를 반복합니다.
    '''
    
    # twiddle is for optimize the kp,ki,kd
    def twiddle(self, setpoint=0):
        best_err = self.cal_error()
        #threshold = 0.001
        #threshold = 1e-09
        threshold = 0.0000000000000000000000000000001 # 1e-31

        # searching by move 1.1x to the target and if go more through the target comeback to -2x
        while sum(self.dp) > threshold:
            for i in range(len(self.p)):
                self.p[i] += self.dp[i]
                err = self.cal_error()

                if err < best_err:  # There was some improvement
                    best_err = err
                    self.dp[i] *= 1.1
                else:  # There was no improvement
                    self.p[i] -= 2*self.dp[i]  # Go into the other direction
                    err = self.cal_error()

                    if err < best_err:  # There was an improvement
                        best_err = err
                        self.dp[i] *= 1.05
                    else:  # There was no improvement
                        self.p[i] += self.dp[i]
                        # As there was no improvement, the step size in either
                        # direction, the step size might simply be too big.
                        self.dp[i] *= 0.95

        print(self.p)

    #########################################################################

    # setpoint is the center and the x_current is where the car is
    # width = 640, so 320 is the center but 318 is more accurate in real
    def pid_control(self, x_current, setpoint=0):
        # print "HHHHHHHHHHHHHHH"
        # print x_current
        self.x = int(x_current)
        self.twiddle()

        error = setpoint - x_current
        p1 = round(self.p[0] * error, 9)
        self.error_sum += error
        i1 = round(self.p[1] * self.error_sum, 9)
        d1 = round(self.p[2] * (error -  self.error_old), 9)
        self.error_old = error
        pid = p1 + i1 + d1
        # print("p : " ,p)
        # print("i : " ,i)
        # print("d : " ,d)
        return pid

"""