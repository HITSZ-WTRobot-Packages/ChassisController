/**
 * @file    JustEncoder.hpp
 * @author  syhanjin
 * @date    2026-03-07
 */
#pragma once

#include "ILoc.hpp"

namespace chassis_loc
{

class JustEncoder final : public ILoc
{
public:
    explicit JustEncoder() : ILoc() {}
    void update(float dt);
};

} // namespace chassis_loc
