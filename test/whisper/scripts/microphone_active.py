#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

def microphone_active_publisher():
    rospy.init_node('microphone_active_publisher', anonymous=True)
    pub = rospy.Publisher('microphone', Bool, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz로 publish할 예정

    microphone_active = True  # 초기 상태 (마이크로폰 비활성화)

    while not rospy.is_shutdown():
        # 여기에서 필요한 조건에 따라 microphone_active 변수를 업데이트
        # 예를 들어, 특정 조건을 만족하면 microphone_active = True로 설정

        # microphone_active 값을 Bool 메시지로 publish
        pub.publish(microphone_active)

        rate.sleep()

if __name__ == '__main__':
    try:
        microphone_active_publisher()
    except rospy.ROSInterruptException:
        pass

