# Unity 构建命令

可复现的 Unity 任务构建、Server build、smoke test 与训练命令由仓库根目录
[`AGENTS.md`](../../AGENTS.md) 维护。命令必须从项目根目录解析 `git` 工作树和
`UNITY_EDITOR`，而不是依赖某台机器的绝对路径。

旧任务目录、过时 build daemon 调用和本机命令记录保留在私有
`Docs/_Archive/RL/Unity_Binary_Build_Commands.zh-CN.md`，不会进入公开发行快照。
