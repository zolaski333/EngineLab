#include <enginelab/runtime/DynoRunArchive.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

namespace {
void require(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
}
}

int main() {
    enginelab::DynoRunArchive archive;
    require(archive.revision() == 0,
        "a fresh archive must have revision zero");

    enginelab::DynoRun first;
    first.id = archive.reserveRunId();
    first.engineName = "first engine";
    first.status = enginelab::DynoRunStatus::completed;
    first.stopReason = enginelab::DynoStopReason::sweepCeilingReached;
    require(first.id == 1 && archive.append(first),
        "the first reserved run must be archived as ID 1");

    enginelab::DynoRun second;
    second.id = archive.reserveRunId();
    second.engineName = "replacement runtime engine";
    second.status = enginelab::DynoRunStatus::cancelled;
    second.stopReason = enginelab::DynoStopReason::operatorCancelled;
    require(second.id == 2 && archive.append(second),
        "a replacement runtime must receive the next stable ID");
    require(archive.revision() == 2,
        "each successful archive mutation must advance its revision once");

    require(!archive.append(first) && archive.revision() == 2,
        "duplicate run IDs must be rejected without replacing evidence");
    auto snapshot = archive.snapshot();
    require(snapshot.size() == 2
            && snapshot[0].id == 1
            && snapshot[1].id == 2
            && snapshot[1].engineName == "replacement runtime engine",
        "the archive snapshot must retain ordered runs across runtimes");

    require(!archive.erase(999) && archive.revision() == 2,
        "deleting an unknown ID must not create a false UI revision");
    require(archive.erase(first.id) && archive.revision() == 3,
        "deleting an existing run must advance the archive revision");
    snapshot = archive.snapshot();
    require(snapshot.size() == 1 && snapshot.front().id == 2,
        "deletion must target only the requested stable ID");

    const auto thirdId = archive.reserveRunId();
    require(thirdId == 3,
        "deleting history must never recycle a run ID");

    std::cout << "PASS: stable dyno archive IDs, revisions and deletion\n";
    return 0;
}
