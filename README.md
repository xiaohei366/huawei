# git使用指南
1. 先将项目`git clone`下来。
2. 一旦需要开发新的功能，就在`remote`的`master`分支的基础上创建一个`feature xxx`分支
3. 本地创建对应的`feature xxx`分支(或者本地创建一个新分支，随后开发测试完毕后合并入主分支)
4. 然后在本地`feature xxx`分支上开发，当开发测试完毕之后，就进行commit--注意填好“备注信息”。
5. 感觉可以上传到云端让别人看了，就`push`到remote的`feature xxx`分支。
6. 在项目主页上发起`pull request`（如果是gitlab则是`merge request`，作用相同），本意是提出将`feature xxx`分支合并入`master`分支的请求
7. 然后你的代码会被review，没通过就本地改，改完之后继续`push`到`remote`（两头都在feature xxx分支），然后负责人继续review你这个PR或者MR，通过之后会将`feature xxx`分支**区别于master的改动**合并入`master`，删除remote的`feature xxx`分支，代表这个功能开发完毕

# 相应的commit格式头
feat：添加新功能或功能改进
fix：修复错误或缺陷
refactor：重构代码而不更改功能
docs：仅更新文档
style：对代码样式进行更改，如缩进或格式化
test：添加或更改测试
chore：更新构建或其他辅助工具的配置

# 如何测试&构建代码
直接在`./SDK/main.cpp`下进行代码书写，当编写完成后：
- 进入`cd ./SDK/build`执行`cmake ..`
- 随后在该路径下执行make生成编译文件
- 返回根目录`cd ../`随后使用判题器判题：
  - 运行根目录文件`sh run_without_gui.sh`直接生成测试结果
  - 运行根目录文件`sh run.sh`生成**可视化结果**&bash打印工作台日志，并可以进入.sh文件调整其输出
    - -f 快速模式，不按照自然时间运行，选手返回控制指令就提前进入下一帧，可选
    - -d 调试模式，不限制选手的初始化和每帧运行时间，方便选手挂载调试器
    - -m 指定地图文件，*必选项*
    - -n 指定选手名字，可选
    - -c 指定选手程序的当前目录，可选
    - -s 指定随机种子，可选
    - -l 指定日志级别，可选
    - -r 指定回放文件存储路径与格式，可选，默认 replay/%Y-%m-%d.%H.%M.%S.rep
    - -h 打印帮助
  
