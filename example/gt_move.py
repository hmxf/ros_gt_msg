import rospy
import os
import logging
import json
from icecream import ic  
from ros_gt_msg.msg import gt_control, Lift_control


# 配置日志
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# 基类，用于消息发布
class BaseCommand:
    """
    基础命令类，用于创建和发布ROS消息。

    参数:
    pub_topic (str): 发布消息的主题名称。
    msg_type: 消息的类型，需要与发布主题的消息类型匹配。

    属性:
    pub_topic (str): 发布消息的主题名称。
    pub: rospy.Publisher对象，用于发布消息。
    param: ReadParam对象，用于读取参数。
    """
    def __init__(self, pub_topic, msg_type):
        # 初始化ROS发布者和参数读取
        self.pub_topic = pub_topic
        self.pub = rospy.Publisher(self.pub_topic, msg_type, queue_size=10)
        self.param = ReadParam()

    def publish_command(self, command_func, *args, **kwargs):
        """
        发布命令消息。

        参数:
        command_func: 执行命令的函数。
        args: 位置参数，传递给command_func。
        kwargs: 关键字参数，传递给command_func。

        无返回值。
        """
        try:
            # 尝试执行命令函数，并发布消息
            ic(*args, **kwargs)  # ic函数用于验证命令执行

            msg = command_func(*args, **kwargs)
            ic(msg)
            self.pub.publish(msg)
            rospy.loginfo(f"指令发布成功: {str(msg)}")
        except Exception as e:
            # 如果有异常发生，则记录错误日志
            rospy.logerr(f"{str(self.pub_topic)}发布指令失败: {str(e)}")

    def sleep(self, seconds):
        """
        延迟执行时间。

        参数:
        seconds (float): 延迟的时间，单位为秒。

        无返回值。
        """
        rospy.sleep(seconds)  # 使用ROS的延迟函数

class BasePlateCommand(BaseCommand):
    """
    BasePlateCommand类负责控制底盘的运动。

    继承自BaseCommand类，初始化时建立与ht_control的连接，并开始移动底盘。
    """
    def __init__(self):
        """
        初始化BasePlateCommand类的实例。
        """
        super().__init__("/GT_Control", gt_control)  # 调用父类构造函数，初始化命令名称和控制对象
        self.move()  # 开始移动基板

    def move(self):
        """
        根据参数设置底盘的运动。
        """
        self.command_setting(self.param.position_param, self.param.position_param.keys())

    def command_setting(self, paramdict, keylist):
        """
        遍历参数字典，根据指定的键值对底盘进行多次运动控制。

        :param paramdict: 包含运动参数的字典。
        :param keylist: 参数字典中的键列表。
        """
        for key in keylist:  # 遍历参数字典中的键
            moveparam = paramdict[key][1:]  # 提取运动参数
            for t in range(paramdict[key][0]):  # 根据参数执行相应的次数
                ic(paramdict[key][0])  # 输入检查函数（未定义）
                self.publish_command(self._base_plate_command, moveparam)  # 发布基板控制命令
                self.sleep(1)  # 休眠1秒

    def _base_plate_command(self, moveparam):
        """
        构造并返回一个控制底盘运动的命令。

        :param moveparam: 运动参数列表，包括模式和坐标。
        :return: 控制基板运动的命令对象。
        """
        control = gt_control()  # 创建控制对象
        control.mode = moveparam[0]  # 设置运动模式
        control.x = moveparam[1]  # 设置x坐标
        control.y = moveparam[2]  # 设置y坐标
        control.stop = moveparam[3]  # 设置z坐标
        return control

class LiftCommand(BaseCommand):
    """
    BasePlateCommand类负责控制底盘的运动。

    继承自BaseCommand类，初始化时建立与ht_control的连接，并开始移动底盘。
    """
    def __init__(self):
        """
        初始化BasePlateCommand类的实例。
        """
        super().__init__("/Lift_Control", Lift_control)  # 调用父类构造函数，初始化命令名称和控制对象
        self.move()  # 开始移动基板

    def move(self):
        """
        根据参数设置底盘的运动。
        """
        self.command_setting(self.param.lift_param, self.param.lift_param.keys())

    def command_setting(self, paramdict, keylist):
        """
        遍历参数字典，根据指定的键值对底盘进行多次运动控制。

        :param paramdict: 包含运动参数的字典。
        :param keylist: 参数字典中的键列表。
        """
        down_command = self._lift_command(paramdict["down"])
        ic(down_command)
        self.pub.publish(down_command)
        self.sleep(10)
        self.pub.publish(down_command)
        self.sleep(15)

        up_command = self._lift_command(paramdict["up"])
        self.pub.publish(up_command)


    def _lift_command(self, moveparam):
        """
        构造并返回一个控制底盘运动的命令。

        :param moveparam: 运动参数列表，包括模式和坐标。
        :return: 控制基板运动的命令对象。
        """
        control = Lift_control()  
        control.mode = moveparam[0] 
        control.data = moveparam[1]  
        control.clear_flag = moveparam[2]

        return control

class ReadParam:
    """
    读取参数配置的类，用于初始化和获取设备运行所需的参数。
    """
    def __init__(self):
        """
        类的构造函数，初始化配置文件路径和参数属性。
        """
        self.configname = "config.json"  # 配置文件名称
        try:
            self.path = self._get_directory_path()  # 尝试获取配置文件的完整路径
        except Exception as e:
            logging.error(f"初始化路径配置失败: {e}")
        
        self._initialize_properties()  # 初始化参数属性

    def _get_directory_path(self):
        """
        获取当前脚本所在目录的上一级目录路径，并拼接配置文件名称返回配置文件的完整路径。
        
        返回:
            str: 配置文件的完整路径
        """
        try:
            current_script_directory = os.path.dirname(os.path.realpath(__file__))  # 获取当前脚本路径
            path = os.path.join(current_script_directory, self.configname)  
            ic(path)  # 信息输出
            return path
        except Exception as e:
            logging.error(f"获取路径时出错: {e}")
            raise

    def _initialize_properties(self):
        """
        初始化类的属性，从配置文件中读取参数。
        """
        try:
            demo_param = self._read_file(self.path)  
        except Exception as e:
            logging.error(f"初始化失败: {e}")
            raise

        # 从读取的参数中，分配到类的属性中
        self.position_param = demo_param["position_param"]
        self.lift_param = demo_param["lift_param"]

        ic(self.lift_param)

    def _read_file(self, file_path) -> object:
        """
        读取并返回文件内容。
        
        参数:
            file_path (str): 文件路径
            
        返回:
            object: 文件内容
        """
        with open(file_path, "r") as j: 
            data = json.load(j)  
        return data


if __name__ == "__main__":
    rospy.init_node("command_node")
    
    # 实例化并运行各个命令发布类
    BasePlateCommand()                  
    LiftCommand()
