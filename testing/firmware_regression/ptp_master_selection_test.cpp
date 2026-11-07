#include <array>
#include <cassert>
#include <cstdio>
#include "../../main/custom/ptp_l2_master_selection.h"

static constexpr int64_t SECOND = 1000000;
static constexpr uint8_t SOURCE_A[10] = {1, 2, 3, 4, 5, 6, 7, 8, 0, 1};
static constexpr uint8_t SOURCE_B[10] = {2, 3, 4, 5, 6, 7, 8, 9, 0, 1};
static constexpr uint8_t SOURCE_C[10] = {3, 4, 5, 6, 7, 8, 9, 10, 0, 1};
using Role = ptp::SelectionRole;

static ptp::MasterDataset dataset(uint8_t identity, uint8_t priority = 128)
{
    ptp::MasterDataset result{};
    result.identity[0] = identity;
    result.priority1 = priority;
    return result;
}

static void ranking()
{
    const auto original = dataset(20);
    assert(ptp::compare_master_datasets(original, original) == 0);
    for (unsigned field = 0; field < 6; ++field)
    {
        auto better = original;
        switch (field)
        {
        case 0: --better.priority1; break;
        case 1: --better.clock_class; break;
        case 2: --better.clock_accuracy; break;
        case 3: --better.variance; break;
        case 4: --better.priority2; break;
        case 5: --better.identity[0]; break;
        }
        assert(ptp::compare_master_datasets(better, original) < 0);
        assert(ptp::compare_master_datasets(original, better) > 0);
    }
    auto preferred = dataset(255, 1);
    auto accurate = dataset(1, 2);
    accurate.clock_class = 6;
    accurate.clock_accuracy = 0x20;
    assert(ptp::compare_master_datasets(preferred, accurate) < 0);
}

static void startup_and_qualification()
{
    ptp::MasterSelection<2> selection(true, 10 * SECOND, 4 * SECOND);
    assert(selection.role(100) == Role::Listening);
    const auto local = dataset(20);
    selection.reset(local, 0);
    assert(selection.role(4 * SECOND - 1) == Role::Listening);
    assert(selection.role(4 * SECOND) == Role::Master);
    const auto worse = dataset(30);
    selection.observe(worse, SOURCE_A, 1, 1, 4 * SECOND);
    selection.observe(worse, SOURCE_A, 2, 1, 5 * SECOND);
    assert(selection.role(5 * SECOND) == Role::Master);
    auto better = dataset(10);
    selection.observe(better, SOURCE_A, 10, 1, 5 * SECOND);
    assert(selection.role(5 * SECOND) == Role::Master);
    selection.observe(better, SOURCE_A, 10, 1, 6 * SECOND);
    selection.observe(better, SOURCE_A, 9, 1, 6 * SECOND);
    assert(selection.role(6 * SECOND) == Role::Master);
    selection.observe(better, SOURCE_A, 11, 1, 7 * SECOND);
    assert(selection.role(7 * SECOND) == Role::Standby);
    selection.observe(better, SOURCE_A, 11, 1, 16 * SECOND);
    assert(selection.role(17 * SECOND - 1) == Role::Standby);
    assert(selection.role(17 * SECOND) == Role::Master);

    selection.observe(better, SOURCE_A, 65535, 1, 18 * SECOND);
    selection.observe(better, SOURCE_A, 0, 1, 19 * SECOND);
    assert(selection.role(19 * SECOND) == Role::Standby);
    better.priority2 = 127;
    selection.observe(better, SOURCE_A, 1, 1, 20 * SECOND);
    assert(selection.role(20 * SECOND) == Role::Standby);
    better.identity[0] = 9;
    selection.observe(better, SOURCE_A, 2, 1, 21 * SECOND);
    assert(selection.role(21 * SECOND) == Role::Master);
    selection.observe(better, SOURCE_A, 3, 1, 22 * SECOND);
    assert(selection.role(22 * SECOND) == Role::Standby);
    selection.observe(worse, SOURCE_A, 4, 1, 23 * SECOND);
    assert(selection.role(23 * SECOND) == Role::Master);
    auto echo = local;
    echo.priority1 = 0;
    selection.observe(echo, SOURCE_B, 1, 1, 23 * SECOND);
    selection.observe(echo, SOURCE_B, 2, 1, 24 * SECOND);
    assert(selection.role(24 * SECOND) == Role::Master);

    selection.observe(better, SOURCE_A, 5, 1, 25 * SECOND);
    selection.observe(better, SOURCE_A, 6, 1, 26 * SECOND);
    assert(selection.role(26 * SECOND) == Role::Standby);
    selection.reset(local, 27 * SECOND);
    assert(selection.role(31 * SECOND - 1) == Role::Listening);
    assert(selection.role(31 * SECOND) == Role::Master);
}

static void intervals_and_capacity()
{
    const auto local = dataset(20);
    const auto first = dataset(10, 100);
    const auto best = dataset(5, 50);
    ptp::MasterSelection<1> selection(true, 10 * SECOND, 0);
    selection.reset(local, 0);
    selection.observe(first, SOURCE_A, 1, 4, 0);
    selection.observe(first, SOURCE_A, 2, 4, 16 * SECOND);
    assert(selection.role(16 * SECOND) == Role::Standby);
    assert(selection.role(64 * SECOND - 1) == Role::Standby);
    assert(selection.role(64 * SECOND) == Role::Master);
    selection.observe(first, SOURCE_A, 3, -8, 65 * SECOND);
    selection.observe(first, SOURCE_A, 4, 5, 65 * SECOND);
    assert(selection.unsupported_announces() == 2);
    assert(selection.role(65 * SECOND) == Role::Master);

    selection.reset(local, 0);
    selection.observe(first, SOURCE_A, 1, 1, 0);
    selection.observe(best, SOURCE_B, 1, 1, SECOND);
    assert(selection.overflow_drops() == 1);
    assert(selection.role(SECOND) == Role::Master);
    selection.observe(best, SOURCE_B, 2, 1, 2 * SECOND);
    assert(selection.role(2 * SECOND) == Role::Standby);
    selection.observe(dataset(1, 1), SOURCE_C, 1, 1, 3 * SECOND);
    assert(selection.overflow_drops() == 2);
    assert(selection.role(3 * SECOND) == Role::Standby);
    assert(selection.role(12 * SECOND) == Role::Master);

    ptp::MasterSelection<0> empty(true, 10 * SECOND, 0);
    empty.reset(local, 0);
    empty.observe(first, SOURCE_A, 1, 1, 0);
    assert(empty.overflow_drops() == 1 && empty.role(0) == Role::Master);
}

static void multiple_masters_and_legacy()
{
    ptp::MasterSelection<2> first(true, 10 * SECOND, 0);
    ptp::MasterSelection<2> second(true, 10 * SECOND, 0);
    const auto a = dataset(10);
    const auto b = dataset(20);
    first.reset(a, 0);
    second.reset(b, 0);
    for (uint16_t sequence = 0; sequence < 2; ++sequence)
    {
        first.observe(b, SOURCE_B, sequence, 1, sequence * SECOND);
        second.observe(a, SOURCE_A, sequence, 1, sequence * SECOND);
    }
    assert(first.role(2 * SECOND) == Role::Master);
    assert(second.role(2 * SECOND) == Role::Standby);
    assert(second.role(11 * SECOND) == Role::Master);

    ptp::MasterSelection<2> several(true, 10 * SECOND, 0);
    several.reset(b, 0);
    const auto preferred = dataset(1, 100);
    const auto backup = dataset(2, 110);
    several.observe(preferred, SOURCE_A, 1, 1, 0);
    several.observe(preferred, SOURCE_A, 2, 1, SECOND);
    several.observe(backup, SOURCE_C, 1, 1, 5 * SECOND);
    several.observe(backup, SOURCE_C, 2, 1, 6 * SECOND);
    assert(several.role(11 * SECOND) == Role::Standby);
    assert(several.role(16 * SECOND) == Role::Master);

    ptp::MasterSelection<2> legacy(false, 10 * SECOND, 4 * SECOND);
    legacy.reset(a, 0);
    assert(legacy.role(0) == Role::Master);
    legacy.observe(b, SOURCE_B, 1, 127, 0);
    assert(legacy.role(0) == Role::Standby);
    legacy.observe(b, SOURCE_B, 1, 127, 9 * SECOND);
    assert(legacy.role(19 * SECOND) == Role::Standby);
    assert(legacy.role(19 * SECOND + 1) == Role::Master);
}

int main()
{
    ranking();
    startup_and_qualification();
    intervals_and_capacity();
    multiple_masters_and_legacy();
    std::puts("PASS: GNSS-only ranking, startup, qualification, sequence wrap/replay, expiry, recovery, capacity and legacy mode");
}
