import rclpy
from rclpy.node import Node
from plansys2_msgs.msg import Plan, PlanItem

class PlanPublisher(Node):
    def __init__(self):
        super().__init__('plan_publisher')
        self.publisher_ = self.create_publisher(Plan, 'executing_plan', 10)
        self.timer = self.create_timer(1.0, self.publish_plan)
        self.get_logger().info('Plan Publisher has been started.')

    def publish_plan(self):
        plan = Plan()
        plan.items = [
            PlanItem(time=0.0, action='(move robot2 kitchen dinning)', duration=5.0),
            PlanItem(time=0.0, action='(askcharge robot1 entrance chargingroom)', duration=5.0),
            PlanItem(time=5.000999927520752, action='(move robot2 dinning entrance)', duration=5.0),
            PlanItem(time=5.000999927520752, action='(charge robot1 chargingroom)', duration=5.0),
            PlanItem(time=10.001999855041504, action='(move robot1 chargingroom kitchen)', duration=5.0),
            PlanItem(time=15.003000259399414, action='(move robot1 kitchen dinning)', duration=5.0),
            PlanItem(time=20.003999710083008, action='(move robot1 dinning bedroom)', duration=5.0),
            PlanItem(time=25.0049991607666, action='(move robot1 bedroom bathroom)', duration=5.0)
        ]
        self.publisher_.publish(plan)
        self.get_logger().info('Plan has been published.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PlanPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
