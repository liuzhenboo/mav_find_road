#include <iostream>
class Cell
{
public:
    Cell();
    Cell(uint8_t class_state, float current_height);
    ~Cell();
    void Update(uint8_t class_state, float current_height);
    float GetZ(int id);

private:
    // 格子的类别，０表示未探索，１表示是路，２表示是障碍物
    uint8_t state;
    float height;
    int seen_times;
};