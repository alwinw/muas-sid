#pragma once

class Inertia
{
public:
    Inertia() = delete;
    Inertia(double m_, double Ix_, double Iy_, double Iz_, double Ixy_, double Iyz_, double Ixz_) : 
        m(m_),
        Ix(Ix_),
        Iy(Iy_),
        Iz(Iz_),
        Ixy(Ixy_),
        Iyz(Iyz_),
        Ixz(Ixz_)
    {
        compute_constants();
    }
    void operator=(const Inertia &inertia)
    {
        m = inertia.m;
        Ix = inertia.Ix;
        Iy = inertia.Iy;
        Iz = inertia.Iz;
        Ixy = inertia.Ixy;
        Iyz = inertia.Iyz;
        Ixz = inertia.Ixz;
        compute_constants();
    }

    double m; 
    double Ix;
    double Iy;
    double Iz;
    double Ixy;
    double Iyz;
    double Ixz;

    double c[10];
private:
    void compute_constants()
    {
        c[0] = Ix*Iz - Ixz*Ixz;
        c[1] = ((Iy - Iz)*Iz - Ixz*Ixz) / c[0];
        c[2] = ((Ix - Iy + Iz)*Ixz) / c[0];
        c[3] = Iz / c[0];
        c[4] = Ixz / c[0];
        c[5] = (Iz - Ix) / Iy;
        c[6] = Ixz / Iy; 
        c[7] = 1 / Iy; 
        c[8] = ((Ix - Iy)*Ix - Ixz*Ixz) / c[0];
        c[9] = Ix / c[0];
    }
};