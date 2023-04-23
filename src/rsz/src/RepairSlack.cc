#include "RepairSlack.hh"

#include "BufferedNet.hh"
#include "db_sta/dbNetwork.hh"
#include "rsz/Resizer.hh"
#include "sta/Corner.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "sta/FuncExpr.hh"
#include "sta/Fuzzy.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/InputDrive.hh"
#include "sta/Liberty.hh"
#include "sta/Parasitics.hh"
#include "sta/PathExpanded.hh"
#include "sta/PathRef.hh"
#include "sta/PathVertex.hh"
#include "sta/PortDirection.hh"
#include "sta/Sdc.hh"
#include "sta/TimingArc.hh"
#include "sta/Units.hh"
#include "utl/Logger.h"

namespace rsz {

using std::abs;
using std::make_shared;
using std::map;
using std::max;
using std::min;
using std::pair;
using std::string;
using std::vector;

using utl::RSZ;

using sta::Clock;
using sta::Corners;
using sta::Edge;
using sta::fuzzyEqual;
using sta::fuzzyGreater;
using sta::fuzzyGreaterEqual;
using sta::fuzzyInf;
using sta::fuzzyLess;
using sta::fuzzyLessEqual;
using sta::INF;
using sta::InputDrive;
using sta::NetConnectedPinIterator;
using sta::PathAnalysisPt;
using sta::PathExpanded;
using sta::PinSeq;
using sta::PinSet;
using sta::Port;
using sta::Unit;
using sta::Units;
using sta::VertexOutEdgeIterator;

RepairSlack::RepairSlack(Resizer* resizer)
    : StaState(),
      logger_(nullptr),
      sta_(nullptr),
      db_network_(nullptr),
      resizer_(resizer),
      corner_(nullptr),
      drvr_port_(nullptr),
      resize_count_(0),
      inserted_buffer_count_(0),
      rebuffer_net_count_(0),
      sizing_count_(0),
      buffering_count_(0),
      splitloads_count_(0),
      buffer_number_(0),
      inverter_number_(0),
      fix_polarity_inv_number_(0),

      min_(MinMax::min()),
      max_(MinMax::max())
{
}

void RepairSlack::init()
{
  logger_ = resizer_->logger_;
  sta_ = resizer_->sta_;
  db_network_ = resizer_->db_network_;
  copyState(sta_);
}

void RepairSlack::repairSlack(bool sizing0,
                              bool buffering0,
                              bool splitload0,
                              int inv_buff_mode0)
{
  sizing = sizing0;
  buffering = buffering0;
  splitload = splitload0;
  inv_buff_mode = inv_buff_mode0;

  float slack_margin = 0.0;
  int max_passes = 1000;
  init();
  logger_->report("command created successful");
}

}  // namespace rsz