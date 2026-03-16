if (Mode_Change_Flag == 1 ||
    Mode_Change_Flag == 2 ||
    Mode_Change_Flag == 3 ||
    Mode_Change_Flag == 4) //设置初始相位
{
    cyc->now_phase = phase;
    Mode_Change_Flag += 1;
    printf("Mode_Change_Flag increased to: %d\n", Mode_Change_Flag);
}
else
{
    Mode_Change_Flag = 0;
    printf("Mode_Change_Flag reset to 0\n");
} 