#pragma once

struct FixedWingDimensions
{
public:
    FixedWingDimensions() = delete;
    FixedWingDimensions(double wing_ref_area, double wing_span, double mean_aerodynamic_chord) : 
        S(wing_ref_area),
        b(wing_span),
        c(mean_aerodynamic_chord)
    {}

    double S; // wing reference area
    double b; // wing span
    double c; // mean aerodynamic chord
};