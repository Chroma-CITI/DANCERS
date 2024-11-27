#pragma once

/**
 * @brief Function used for flocking computation, see curve in Vásárhelyi 2018 Fig.6.
 */
double SigmoidLin(const double r, const double a, const double p)
{
    if (r <= 0)
    {
        return 0;
    }
    else if (r * p > 0 && r * p < a / p)
    {
        return r * p;
    }
    else
    {
        return std::sqrt(2 * a * r - std::pow(a, 2) / std::pow(p, 2));
    }
}


/**
 * @enum Indicates the role of an agent
 */
enum AgentRoleType
{
    Iddle=0,
    Mission
};