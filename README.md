# sneaky-sky-stalker
## 0.依赖
- ROS Noetic
## 1.配置ssh key
首先在本地电脑配置ssh key,使得可以通过ssh key登录到github账户,这在clone仓库以及push代码的时候会有用
```shell
cd ~/.ssh
ls
#看是否存在 id_rsa 和 id_rsa.pub文件，如果存在，说明已经有SSH Key
#如果没有则需要使用如下指令创建
ssh-keygen -t rsa -C "xxx@xxx.com"
#执行后一直回车即可

#有了SSH KEY之后
cd ~/.ssh
cat id_rsa.pub
```
复制打印出来的内容, 至[个人sshkey管理页面](https://github.com/settings/keys),新建ssh key,并填入复制的内容
执行
```
ssh -T git@github.com
```
若显示出`Hi, xxx! You've successfully.....`则表示配置成功
至此在push代码的时候远程已经知道你是谁, 但是本地的git还不知道你是谁,还需要配置
```shell
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
```
## 2.克隆仓库并编译
```shell
#到home目录下
cd ~
git clone git@github.com:FranCOOlin/sneaky-sky-stalker.git
cd ~/sneaky-sky-stalker
catkin clean
catkin build
echo "source ~/sneaky-sky-stalker/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## 3.开发自己的package
首先创建自己的分支
```shell
git checkout -b your-branch-name
```
不同的功能在不同的package内实现,类似的功能或内部依赖的功能在同一package内实现,创建package的指令类似如下
```shell
cd ~/sneaky-sky-stalker/src
catkin_create_pkg your_pkg roscpp rospy std_msgs sensor_msgs
```
最后提交修改
```shell
git add . && git commit -m "..."
git push origin your-branch-name
```

# sneaky-sky-stalker
## 1.配置ssh key
首先在本地电脑配置ssh key,使得可以通过ssh key登录到github账户,这在clone仓库以及push代码的时候会有用
## 2.克隆仓库并编译
```shell
#到home目录下
cd ~
git clone git@github.com:FranCOOlin/sneaky-sky-stalker.git
cd ~/sneaky-sky-stalker
catkin clean
catkin build
echo "source ~/sneaky-sky-stalker/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## 3.开发自己的package
首先创建自己的分支
```shell
git checkout -b your-branch-name
```
不同的功能在不同的package内实现,类似的功能或内部依赖的功能在同一package内实现,创建package的指令类似如下
```shell
cd ~/sneaky-sky-stalker/src
catkin_create_pkg your_pkg roscpp rospy std_msgs sensor_msgs
```
最后提交修改
```shell
git add . && git commit -m "..."
git push origin your-branch-name
```
