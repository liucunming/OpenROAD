
#pragma once

#include "db_sta/dbSta.hh"
#include "sta/MinMax.hh"
#include "sta/StaState.hh"
#include "utl/Logger.h"
#include "utility"

using std::pair;

namespace sta {
class PathExpanded;
}

namespace rsz {

class Resizer;

using std::vector;

using utl::Logger;

using sta::Corner;
using sta::dbNetwork;
using sta::dbSta;
using sta::DcalcAnalysisPt;
using sta::LibertyCell;
using sta::LibertyPort;
using sta::LibertyPortSet;
using sta::LibertyCellSeq;
using sta::Delay;
using sta::InstanceSeq;
using sta::MinMax;
using sta::Net;
using sta::PathExpanded;
using sta::PathRef;
using sta::Pin;
using sta::PinSet;
using sta::Slack;
using sta::StaState;
using sta::TimingArc;
using sta::Vertex;
using sta::FuncExpr;
using sta::Instance;
using rmp::Restructure;
using gpl::Replace;
using odb::Point;


class BufferedNet;
enum class BufferedNetType;
typedef std::shared_ptr<BufferedNet> BufferedNetPtr;
typedef vector<BufferedNetPtr> BufferedNetSeq;

class RepairSlack : StaState
{
 public:
  RepairSlack(Resizer* resizer);
  void repairSlack(bool sizing0,
             bool buffering0,
             bool splitload0,
             int inv_buff_mode0 );

 private:
  void init();
  bool repairSlack(PathRef& path, Slack path_slack);
  
  Slack slackPenalized(BufferedNetPtr bnet,int index = -1);
  BufferedNetSeq rebufferBottomUp(BufferedNetPtr bnet, int level,int tmp);
  float pinCapacitance(const Pin* pin, const DcalcAnalysisPt* dcalc_ap);
  BufferedNetSeq addWireAndBuffer(BufferedNetSeq Z,
                                  BufferedNetPtr bnet_wire,
                                  int level);
  float bufferInputCapacitance(LibertyCell* buffer_cell,
                               const DcalcAnalysisPt* dcalc_ap);
  int rebufferTopDown(BufferedNetPtr choice, Net* net, int level, bool polarity);
  BufferedNetPtr rebufferTopDown(BufferedNetPtr choice, bool polarity);
  int fanout(Vertex* vertex);
  bool hasTopLevelOutputPort(Net* net);

  void buffer_fanout(PathRef* drvr_path,
                     int drvr_index,
                     Slack drvr_slack,
                     PathExpanded* expanded);

  void splitLoads(PathRef* drvr_path, Slack drvr_slack);

  int rebuffer(const Pin* drvr_pin,
                       PathRef* drvr_path,
                       int drvr_index,
                       PathExpanded* expanded);

  int bufferin_only(const Pin* drvr_pin);

  pair<BufferedNetSeq,BufferedNetSeq> addWireAndInv(pair<BufferedNetSeq,BufferedNetSeq> Z,
                              BufferedNetPtr bnet_wire);
  int count_junction(BufferedNetPtr bnet);

  pair<BufferedNetSeq,BufferedNetSeq> rebufferBottomUp(BufferedNetPtr bnet);
  BufferedNetSeq addWire(BufferedNetSeq Z,BufferedNetPtr bnet_wire);
  BufferedNetSeq addBuffoInv(BufferedNetSeq Z,BufferedNetPtr bnet_wire,LibertyCellSeq buffoinv);
  void purnSolution(BufferedNetSeq &Z);
  void purnBuffInv(BufferedNetSeq &Z);
  Pin* getoutpin(Instance* load_inst);
  PinSet getinPinSet(Instance* inst);
  float gateDelayDiffer(Instance* inst,LibertyCell* demorgan_cell,LibertyPort *inst_in_port,Instance *prev_inst);
  void removeInstance(Instance* inst);


  //opt_no_phy
  void no_phy_splitLoads(Pin* drvr_pin,PathRef* drvr_path, Slack drvr_slack);
  BufferedNetPtr make_tree(BufferedNetSeq sinks);
  BufferedNetSeq initSortSink(Vertex* drvr_vertex);
  void opt_no_phy();
  bool opt_no_phy(PathRef& path, Slack path_slack);
  int buffering_no_phy(PathRef *drvr_path,
                        int drvr_index,
                        Slack drvr_slack,
                        PathExpanded *expanded);
  BufferedNetPtr joint_the_junc(BufferedNetSeq sinkgroup_nocritical);
  pair<BufferedNetSeq,BufferedNetSeq> addInv_no_phy(pair<BufferedNetSeq,BufferedNetSeq> Z,
                              BufferedNetPtr bnet_wire);
  BufferedNetSeq update_sink_required(BufferedNetPtr best_option, bool &change);
  BufferedNetPtr findBestOption(BufferedNetPtr tree);
  Slack topDownSetLevel(BufferedNetPtr choice,int level,Delay AT);
  void no_phy_resyn();
  BufferedNetPtr add_location_buffer(BufferedNetPtr z);
  void updateTiming(BufferedNetPtr best_option);
  float get_fanout_cap(Pin* outpin);
  void sortSinks(BufferedNetSeq & sinks);
  BufferedNetPtr bufferingIter(Vertex* drvr_vertex,Net* net);
  InstanceSeq bufferset_;
  int max_iter_ = 5;
  int iter_num_ = 0;
  vector<BufferedNetSeq> sink_sorts_;
  float total_cap_ = 0;
  bool DP_bf_debug_ = 0;
  int group_name_ = 0;
  bool final_group_ = 0;
  bool tmp_ = 1;
  int buffering_max_iter_ = 3;

  
  sta::Delay worst_required_ = 0;

  Logger* logger_;
  dbSta* sta_;
  dbNetwork* db_network_;
  Resizer* resizer_;
  const Corner* corner_;
  LibertyPort* drvr_port_;
  Restructure* rmp_;
  Replace* gpl_;

  int resize_count_;
  int inserted_buffer_count_;
  int rebuffer_net_count_;

  int sizing_count_;
  int buffering_count_;
  int splitloads_count_;

  int buffer_number_;
  int inverter_number_;
  int fix_polarity_inv_number_;
  int resyn_count_;

  const MinMax* min_;
  const MinMax* max_;
  static constexpr int repair_slack_decreasing_slack_passes_allowed_ = 50;
 
  static constexpr int split_load_fanout_min_ = 8;
  static constexpr double rebuffer_buffer_penalty_ = .01;

  static constexpr int high_fanout = 50;

  static constexpr int max_fanout_ = 20;

  // mode 1  :split_loads_by_average_fanout
  // mode 2  :split_loads_by_fanout_limit
  // mode 3  :split_loads_origin
  //static constexpr int split_loads_mode = 3;
  // mode 1  :buffering only
  // mode 2  :buffering and sizing drive
  // mode 3  :pFOM rebuffer
  int buffering_mode = 3;

  // mode 1  :buffer only
  // mode 2  :inverter only
  // mode 3  :inverter and buff
  int inv_buff_mode = 3;

  static constexpr int rebuffer_fanout_max_ = 10000000;
  bool sizing = 1;
  bool buffering = 1;
  bool splitload = 1;
  bool no_phy_ = 1;
  bool resyn_ = 1;

  // repair_polarity_ = 1 -> to repair_polarity
  // repair_polarity_ = 0 -> do not repair_polarity
  bool repair_polarity_ = 1;
  bool invresyn_ = 0;

  

  float Cbuffer = 0.0;
  float Rbuffer = 0.0;
  sta::Delay theta_ = 1e-10;
  int p_ = 5;
  int rebuffer_times = 0;

  int maybe_inv_ = 0;

  int resyn_itera_ = 0;
  int total_itera_ = 0;

  sta::Arrival root_arrival_ = 0.0;
  Point loca_ = Point(0,0);

};

}  // namespace rsz
