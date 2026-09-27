/*
 * Flash 写入调度：擦/写各约 4.5ms 期间 CPU 停顿、无法回 GoodCRC，
 * 只在两端 PD 都空闲一段时间、且没有进行中的协商时，每次执行一个页操作
 */
#pragma once

void store_process(void);
