#pragma once

namespace control_lib
{
    /**
     * @brief 线性插值函数
     * @tparam T 数据类型
     * @param a 起始值
     * @param b 结束值
     * @param t 插值系数 [0,1]
     * @return 插值结果
     */
    template <typename T>
    T lerp(const T &a, const T &b, float t)
    {
        return a + (b - a) * t;
    }

    /**
     * @brief 二次外推函数
     * @tparam T 数据类型
     * @param p0 第一个点
     * @param p1 第二个点
     * @param p2 第三个点
     * @param t 外推系数
     * @return 外推结果
     */
    template <typename T>
    T quadraticExtrapolate(const T &p0, const T &p1, const T &p2, float t)
    {
        return p0 + (p1 - p0) * t + (p2 - 2 * p1 + p0) * t * t / 2;
    }

    // 可添加更多数学工具函数...
}
