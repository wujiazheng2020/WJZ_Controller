概述：开源机器人控制集合库，由于是我一个人写的刚上传，不可避免有些BUG，故暂时推荐用于学习使用，如果发现BUG或者需要我更新新的算法,或者需要技术合作,可以联系我修改或者pull request，如果觉得这个项目好请给我颗星星或者branch，这个项目会不断更新，谢谢大家！</br>
Overview: The open source robot planning collection library, because I wrote it by myself and just uploaded it, it is inevitable that there are some BUGs, so it is temporarily recommended for learning. If you find a bug,or need me to update the new algorithm,or need technical cooperation, you can contact me to modify or pull the request. If you think this project is good, please give me a star or branch. This project will be constantly updated, thank you!

BLOG:https://blog.csdn.net/qq_38588806</br>
联系方式 (contact)：</br>
163：wujiazheng2020@163.com</br>
gmail: wujiazheng2020@gmail.com</br>
QQ群：710805413</br>
算法大概介绍见(algorithm introduction):/doc</br>
# Dependece
1.eigen3 一般ubuntu自带(ubuntu installed already)</br>
2.ros: 为了可视化(just for visualization)</br>
3.ipopt 仅MPC需要(just for MPC,if no use,ignore)</br>
4.cppad 仅MPC需要(just for MPC,if no use,ignore)</br>
4.Udacity MPC: MPC.cc MPC.h 仅MPC需要(just for MPC,if no use,ignore)</br>

# Notice:
一方面因为版权问题，另一方面，考虑到ipopt及其第三方的安装会让很多linux新手难受，HSL还需要邮件，很多人只需要差分驱动机器人比如扫地机器人的PID，故为了编译方便，Udacity MPC文件需要自己从github下载，我已经写好了框架，有了那个文件，ipopt那些，把'//'取消稍作修改即可。
On the one hand, because of the copyright problem, on the other hand, considering that the installation of ipopt and its third party will make many linux novices uncomfortable, HSL also needs email, many people only need differential drive robots such as PID of sweeping robot, so in order to compile conveniently, udacity MPC files need to be downloaded from GitHub by yourself. I have already written the framework. With that file, ipopt files, just delete '/ /' and make some changes is OK.

#Contents
##1 Algorithm
注：本程序所用轨迹为从停车场-4上升到-2曾的轨迹
### 1.1 PID_DIFF
![PID_DIFF](https://github.com/wujiazheng2020/WJZ_Controller/blob/master/picture/PID_DIFF.png)
实际上很多人说要双环PID，但是就实际测试情况来说，单环这种对大部分机器人够了，而双环机器人，大部分厂家的生产标准不能到大公司的标准，换了机器人要重调试很麻烦，实际过程中对大型机器人，PID需要一些trick，比如停止时，轨迹处理这些，这些不属于算法内容，可以自己去探索。
### 1.2 PID_ACM
![PID_ACM](https://github.com/wujiazheng2020/WJZ_Controller/blob/master/picture/PID_ACM.png)
老实说实际过程中，做成双环效果似乎不如单环，但大家可以自行设置再加一环，会单环加环也很容易，对无人车，需要一些trick才能实际运行，比如时间不同，停车处理，各种逻辑的处理，这些不属于算法了，可以自行加上。
### 1.3 MPC_ACM
![MPC_ACM](https://github.com/wujiazheng2020/WJZ_Controller/blob/master/picture/MPC_ACM.png)
我曾经有一段时间就是专门做MPC，可以实现下停车场并倒车入库，上旋转的坡，车道保持跟随各种，但是MPC运用时是要考虑时间同步的，时间对MPC特别重要，因为MPC是等时间间隔预测的，另外需要一些轨迹处理，很多停车，倒车逻辑，处理规划路线逻辑，这些根据不同车有不同处理方法，不属于算法本身，可以自己探索。
##2 Time sync
设定一个可信时间区间，对早到的时间，要按运动学补偿,迟到超过一定时间的数据，直接按运动学推算，单独见time_sync/time_sync.txt

