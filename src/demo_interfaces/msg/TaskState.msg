# 任务状态消息（任务流程状态机）
# state 字段取值见下方常量

uint8 IDLE=0            # 空闲，等待启动
uint8 PICK=1            # 从料盘取料
uint8 PLACE=2           # 上料到加工台
uint8 WAIT_MACHINING=3  # 等待加工完成
uint8 RETRIEVE=4        # 从加工台取回工件
uint8 RETURN=5          # 放回料盘
uint8 DONE=6            # 单次上下料循环完成
uint8 WAIT_STATION=7    # 工位忙碌，等待空闲（拓展）
uint8 TIMEOUT=8         # 加工超时警告（拓展）
uint8 ABORTED=9         # 任务中止（取消/异常）

uint8 state             # 当前状态（取值见上）
uint8 slot              # 当前操作的料盘槽位（从 0 开始）
string detail           # 状态说明文字（中文，便于日志与演示）
builtin_interfaces/Time stamp
