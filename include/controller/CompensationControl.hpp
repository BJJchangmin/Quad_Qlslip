#ifndef Compensation_Control_HPP
#define Compensation_Control_HPP

#include "RobotLeg.hpp"
#include <cmath>

template <typename T>
class CompensationControl
{

  private:
    RobotLeg<T> & robot_;

    /**
     * @brief Parameter for Compensation Control
     * todo: Add Trunk gravity Compensation -> using trunk CoM
     * todo: Make new file filter.cpp for filter
     * ! Be cafreful to check what is 4 or 1
     * @param [4]: FL, FR, RL, RR
     * @param [1]: 공통 변수
     *
     *
     */
    T g, l, M_d, pi, sampling_time;

    T m1, m2, l1, l2, l1_c, l2_c;
    T I1_zz, I2_zz;
    T G_m, G_b;
    T C_I;
    T I_m, I_b;

    T th_m[4], th_b[4], th_br[4];
    T dth_m[4], dth_b[4], dth_br[4];

    //Calculate fofr motor accelelration
    T dth_m_old[4], dth_b_old[4], ddth_m_old[4], ddth_b_old[4];
    T ddth_m[4], ddth_b[4]; // motor acceleration

    Vec3<T> body_com;
    Vec3<T> foot_pos[4];
    Vec3<T> vec_body2foot[4];
    Vec2<T> body_weight;
    Mat2<T> cal_Mat[2];
    Vec2<T> r_grf[4];

    Vec2<T> gravity_compensation_joint_des_[4];
    Vec2<T> coriollis_compensation_joint_des_[4];
    Vec2<T> inertia_decoupling_joint_des_[4];
    Vec2<T> inertia_modulation_joint_des_[4];
    Vec2<T> compensation_joint_des_[4];
    Vec2<T> Trunk_mass_compensation_joint_des_[4];


  public:
    CompensationControl(RobotLeg<T> & robot);

    void state_update(int Leg_num);
    T tustin_derivative(T input, T input_old, T output ,T output_old, T cut_off);
    void Gravity_compensation(int Leg_num);
    void Coriollis_compensation(int Leg_num);
    void Inertia_Decoupling(int Leg_num);
    void Inertia_modulation(int Leg_num);
    void Trunk_mass_compensation(mjData * d);

    void compensation_control(int Leg_num);








};



#endif //Compensation_Control_HPP

