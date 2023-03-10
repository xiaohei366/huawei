# git使用指南
1. 先将项目`git clone`下来。
2. 一旦需要开发新的功能，就在`remote`的`master`分支的基础上创建一个`feature xxx`分支
3. 本地创建对应的`feature xxx`分支(或者本地创建一个新分支，随后开发测试完毕后合并入主分支)
4. 然后在本地`feature xxx`分支上开发，当开发测试完毕之后，就进行commit--注意填好“备注信息”。
5. 感觉可以上传到云端让别人看了，就`push`到remote的`feature xxx`分支。
6. 在项目主页上发起`pull request`（如果是gitlab则是`merge request`，作用相同），本意是提出将`feature xxx`分支合并入`master`分支的请求
7. 然后你的代码会被review，没通过就本地改，改完之后继续`push`到`remote`（两头都在feature xxx分支），然后负责人继续review你这个PR或者MR，通过之后会将`feature xxx`分支**区别于master的改动**合并入`master`，删除remote的`feature xxx`分支，代表这个功能开发完毕

# 相应的commit格式头
1. feat：添加新功能或功能改进
2. fix：修复错误或缺陷
3. refactor：重构代码而不更改功能
4. docs：仅更新文档
5. style：对代码样式进行更改，如缩进或格式化
6. test：添加或更改测试
7. chore：更新构建或其他辅助工具的配置
