# 机床（加工台）工位状态消息

uint8 READY=0      # 空闲就绪，可上料
uint8 BUSY=1       # 忙碌（初始占用/换型中）
uint8 MACHINING=2  # 正在加工工件
uint8 DONE=3       # 加工完成，等待取回

uint8 state         # 当前工位状态（取值见上）
float32 progress    # 加工进度 0~100（仅 MACHINING 状态有意义）
string detail       # 状态说明文字
builtin_interfaces/Time stamp
