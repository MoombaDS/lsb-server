/*
===========================================================================

  Copyright (c) 2026 LandSandBoat Dev Teams

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses/

===========================================================================
*/

#include "map/ai/helpers/pathfind/path_owner.h"
#include "map/ai/helpers/pathfind/pathfind.h"
#include "map/navmesh/navmesh.h"
#include "map/roam_region.h"

#include "common/utils.h"
#include "common/xirand.h"

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>

namespace
{

// flat ground everywhere: every path is the straight line to its end
class FlatNavMesh final : public NavMesh
{
public:
    auto findPath(const position_t&, const position_t& end, float, AvoidLinks) -> Maybe<PathResult> override
    {
        return PathResult{ { pathpoint_t{ end, 0s, false } }, false };
    }

    auto findRandomPosition(const position_t& start, float) const -> Maybe<position_t> override
    {
        return start;
    }

    auto validPosition(const position_t&) const -> bool override
    {
        return true;
    }

    auto findClosestValidPoint(const position_t& position) const -> Maybe<position_t> override
    {
        return position;
    }

    auto findFurthestValidPoint(const position_t&, const position_t& endPosition) const -> Maybe<position_t> override
    {
        return endPosition;
    }

    auto snapToValidPosition(position_t&) const -> void override
    {
    }

    auto moveAlongSurface(const position_t&, const position_t& end, position_t& result) const -> bool override
    {
        result = end;
        return true;
    }
};

class StubOwner final : public pathfind::PathOwner
{
public:
    StubOwner(position_t position, NavMesh& navMesh)
    : position_(position)
    , navMesh_(navMesh)
    {
    }

    auto position() -> position_t& override
    {
        return position_;
    }

    auto position() const -> const position_t& override
    {
        return position_;
    }

    auto navMesh() -> NavMesh& override
    {
        return navMesh_;
    }

    auto markPositionDirty() -> void override
    {
    }

    auto baseSpeed() const -> uint8 override
    {
        return 40;
    }

    auto updateSpeed(bool) -> uint8 override
    {
        return 40;
    }

    auto isMobEntity() const -> bool override
    {
        return true;
    }

    auto isRoaming() const -> bool override
    {
        return true;
    }

    auto inWater() const -> bool override
    {
        return false;
    }

    auto battleTargetPosition() const -> const position_t* override
    {
        return nullptr;
    }

    auto onPathPoint() -> void override
    {
    }

    auto onPathComplete() -> void override
    {
    }

    auto name() const -> const std::string& override
    {
        return name_;
    }

    auto id() const -> uint32 override
    {
        return 1;
    }

    auto hitboxRadius() const -> float override
    {
        return 0.0f;
    }

private:
    position_t  position_;
    NavMesh&    navMesh_;
    std::string name_{ "stub" };
};

auto squareWithHole() -> RoamRegion
{
    const RoamRegion::Ring outer{ { 0.0f, -10.0f, 0.0f }, { 100.0f, -10.0f, 0.0f }, { 100.0f, -10.0f, 100.0f }, { 0.0f, -10.0f, 100.0f } };
    const RoamRegion::Ring hole{ { 40.0f, -10.0f, 40.0f }, { 60.0f, -10.0f, 40.0f }, { 60.0f, -10.0f, 60.0f }, { 40.0f, -10.0f, 60.0f } };

    return RoamRegion(outer, { hole });
}

// every yalm of the straight leg from `from` to `to` lies in the region
auto legStaysInside(const RoamRegion& region, const position_t& from, const position_t& to) -> bool
{
    const float length = distance(from, to, true);
    for (int step = 0; static_cast<float>(step) * 0.5f <= length; ++step)
    {
        const float along = static_cast<float>(step) * 0.5f;
        const float t     = length > 0.0f ? along / length : 0.0f;
        if (!region.contains(from.x + (to.x - from.x) * t, from.z + (to.z - from.z) * t))
        {
            return false;
        }
    }

    return region.contains(to.x, to.z);
}

} // namespace

TEST_CASE("pathfind: a roam leg never crosses out of its region", "[pathfind][region]")
{
    const auto  region = squareWithHole();
    FlatNavMesh navMesh;

    // hard against the hole, where a drawn step is most likely to want to cross it
    for (const auto& start : { position_t{ 39.0f, -10.0f, 50.0f, 0, 0 }, position_t{ 50.0f, -10.0f, 39.0f, 0, 0 }, position_t{ 2.0f, -10.0f, 2.0f, 0, 0 } })
    {
        for (int i = 0; i < 300; ++i)
        {
            auto      owner    = std::make_unique<StubOwner>(start, navMesh);
            auto*     ownerPtr = owner.get();
            CPathFind pathFind(std::move(owner));

            if (!pathFind.RoamAround(start, 30.0f, 1, 1, xi::RoamFlag::None, &region))
            {
                continue;
            }

            REQUIRE(legStaysInside(region, ownerPtr->position(), pathFind.GetDestination()));
        }
    }
}

TEST_CASE("pathfind: a region mob that cannot sample a leg walks back onto its region", "[pathfind][region]")
{
    const auto  region = squareWithHole();
    FlatNavMesh navMesh;

    // outside the outline every sample fails
    const position_t stranded{ -10.0f, -10.0f, 50.0f, 0, 0 };
    auto             owner    = std::make_unique<StubOwner>(stranded, navMesh);
    auto*            ownerPtr = owner.get();
    CPathFind        pathFind(std::move(owner));

    // the failed sample turns into a walk to a point inside, not cut at the outline
    REQUIRE(pathFind.RoamAround(stranded, 10.0f, 1, 1, xi::RoamFlag::None, &region));
    const auto destination = pathFind.GetDestination();
    CHECK(region.contains(destination.x, destination.z));
    CHECK_FALSE(region.contains(ownerPtr->position().x, ownerPtr->position().z));
}

TEST_CASE("pathfind: roam steps stay on the Manaclipper deck", "[pathfind][region]")
{
    // Manaclipper deck outline.
    const RoamRegion::Ring deck{
        { -11.15f, -3.0f, -5.81f }, { -9.66f, -3.0f, -10.61f }, { -9.78f, -3.0f, -15.83f }, { -4.79f, -3.0f, -18.21f }, { 0.13f, -3.0f, -16.25f }, { 4.65f, -3.0f, -17.67f }, { 9.99f, -3.0f, -15.77f }, { 9.51f, -3.0f, -10.79f }, { 11.18f, -3.0f, -6.05f }, { 10.05f, -3.0f, 3.86f }, { 0.04f, -3.0f, 1.60f }, { -9.95f, -3.0f, 3.68f }
    };
    const RoamRegion region(deck, {});
    FlatNavMesh      navMesh;
    const position_t anchor{ 0.0f, -3.0f, -8.0f, 0, 0 };
    for (uint64 seed = 0; seed < 200; ++seed)
    {
        xirand::rng().seed(seed);
        auto      owner = std::make_unique<StubOwner>(anchor, navMesh);
        auto*     actor = owner.get();
        CPathFind path(std::move(owner));
        unsigned  movedFrames     = 0;
        unsigned  lateMovedFrames = 0;
        for (int frame = 0; frame < 500; ++frame)
        {
            if (!path.IsFollowingPath())
            {
                path.RoamAround(anchor, 30.0f, 1, 4, xi::RoamFlag::None, &region);
            }
            const auto before = actor->position();
            path.FollowPath(timer::now() + std::chrono::milliseconds(frame * 400));
            const auto& p     = actor->position();
            const bool  moved = p.x != before.x || p.z != before.z;
            movedFrames += moved;
            lateMovedFrames += frame >= 400 && moved;
            INFO("seed=" << seed << " frame=" << frame << " x=" << p.x << " z=" << p.z);
            REQUIRE(region.contains(p.x, p.z));
        }
        REQUIRE(movedFrames > 0);
        REQUIRE(lateMovedFrames > 0);
    }
}

TEST_CASE("pathfind: roam steps do not cross holes", "[pathfind][region]")
{
    const auto       region = squareWithHole();
    FlatNavMesh      navMesh;
    const position_t anchor{ 39.0f, -10.0f, 50.0f, 0, 0 };
    for (uint64 seed = 0; seed < 50; ++seed)
    {
        xirand::rng().seed(seed);
        auto      owner = std::make_unique<StubOwner>(anchor, navMesh);
        auto*     actor = owner.get();
        CPathFind path(std::move(owner));
        unsigned  movedFrames     = 0;
        unsigned  lateMovedFrames = 0;
        for (int frame = 0; frame < 500; ++frame)
        {
            if (!path.IsFollowingPath())
            {
                path.RoamAround(anchor, 30.0f, 1, 4, xi::RoamFlag::None, &region);
            }
            const auto before = actor->position();
            path.FollowPath(timer::now() + std::chrono::milliseconds(frame * 400));
            const auto& p     = actor->position();
            const bool  moved = p.x != before.x || p.z != before.z;
            movedFrames += moved;
            lateMovedFrames += frame >= 400 && moved;
            INFO("seed=" << seed << " frame=" << frame << " x=" << p.x << " z=" << p.z);
            REQUIRE(region.contains(p.x, p.z));
        }
        REQUIRE(movedFrames > 0);
        REQUIRE(lateMovedFrames > 0);
    }
}

TEST_CASE("pathfind: a mob outside its region walks back in", "[pathfind][region]")
{
    const RoamRegion::Ring outer{ { 0.0f, -10.0f, 0.0f }, { 100.0f, -10.0f, 0.0f }, { 100.0f, -10.0f, 100.0f }, { 0.0f, -10.0f, 100.0f } };
    const RoamRegion       region(outer, {});
    FlatNavMesh            navMesh;
    const position_t       stranded{ -10.0f, -10.0f, 50.0f, 0, 0 };
    xirand::rng().seed(0);
    auto      owner = std::make_unique<StubOwner>(stranded, navMesh);
    auto*     actor = owner.get();
    CPathFind path(std::move(owner));
    REQUIRE(path.RoamAround(stranded, 10.0f, 1, 1, xi::RoamFlag::None, &region));
    const auto destination = path.GetDestination();
    REQUIRE(region.contains(destination.x, destination.z));
    REQUIRE_FALSE(region.contains(actor->position().x, actor->position().z));
    for (int frame = 0; frame < 500 && path.IsFollowingPath(); ++frame)
    {
        path.FollowPath(timer::now() + std::chrono::milliseconds(frame * 400));
    }
    CHECK(region.contains(actor->position().x, actor->position().z));
    CHECK(isWithinDistance(actor->position(), destination, 0.1f, true));
}
