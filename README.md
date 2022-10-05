# FANUC ストリーミング制御用ROSパッケージの説明
## 起動方法

下記のコマンドを実行することで、rvizの画面が立ち上がり、動作コマンドを受け取り、ロボットを制御することが可能となります。  
この時、ロボットのティーチングペンダントをOFFにして「AUTO」モードにしておく必要があります。
```
$ roslaunch robot_trajectory_controller robot_trajectory_controller.launch
```

なお、ロボットのIPアドレスや、ファナックドライバのライセンスファイルの場所に変更がある場合は、この起動ファイル（ローンチファイル）の内容を修正する必要があります。
```xml robot_trajectory_controller.launch {.line-number}
<launch>
    <arg name="ip" default="192.168.1.100"/>
    <arg name="robot_type" default="CRX-10iA"/>
    <arg name="license" default="/home/tokano/license.data"/>
```
上記の２行目のIPアドレスをロボットコントローラのIPアドレスにしてください。  
４行目のライセンスファイルのパスをお使いの環境に合わせるようようにしてください。  

この他に、ロボットコントローラを含めて特別な設定はありません。

## プログラム方法

ロボットを制御するためには、ロボットを制御するサーバープログラムに対して、コマンドを送信するためのクライアントプログラムが必要になります。
クライアントプログラムの例として、controller_client.pyが本パッケージに含まれおりますので、そちらを参照すれば、容易にクライントプログラムを作成することが可能です。

下記の例は、そのサンプルクライントプログラムの冒頭の部分の抜粋になります。

```python controller_client.py {.line-number}
import rospy
import geometry_msgs.msg
from robot_trajectory_controller.srv import ServerCommand, ServerCommandRequest

def main():
    rospy.init_node('robot_trajectory_client', anonymous=True)
    rospy.wait_for_service('server_command')
    try:
        client = rospy.ServiceProxy('server_command', ServerCommand)
        req = ServerCommandRequest()
        req.cmd_id = 1
        req.type = ServerCommandRequest.GET_CURRENT_STATE
        robot_state = client(req)
        rospy.loginfo(robot_state)
```
ServerCommandRequestを生成し、そこに必要な値をセットし、上記の例
clinet(req)として実行することで、サーバープログラムに必要な情報を渡すことでロボットの制御が可能になります。

なお、ServerCommandのtypeについては、ServerCommand.srvの内容（下記）を参照してください。

```
uint8 GET_CURRENT_STATE=0
uint8 MOVE_JOINT=1
uint8 MOVE_POSE=2
uint8 CANCEL_GOAL=3
uint8 GET_DI=4
uint8 GET_RI=5
uint8 SET_DO=6
uint8 SET_RO=7
uint8 SET_VELOCITY=8
uint8 SET_ACCELERATION=9
int32 cmd_id
int32 type
float64[] target_joints
geometry_msgs/Pose target_pose
uint32 io_address
uint32 io_value
float64 value
---
int32 cmd_id
bool is_moving
float64[] current_joints
geometry_msgs/Pose current_pose
uint32 io_value
```

最後に、本件に関するお問い合わせは下記のメールアドレスにてご連絡ください。  
E-mail : token.okano@gmail.com

以上