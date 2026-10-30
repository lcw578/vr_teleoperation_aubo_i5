// Compile the production binding unchanged against an offline SDK double.
#define AUBO_FOLLOW_TEST_BEFORE_PUSH_READ fake_follow::before_push_read
#define AUBO_FOLLOW_TEST_PUSH_TIMESTAMP fake_follow::push_timestamp
#include "../../teleop/aubo/native/follow.cpp"
PYBIND11_MODULE(_aubo_follow_controls,m){
 m.def("block_query",[](bool v){fake_follow::b_entered=false;fake_follow::block_b=v;});
 m.def("query_entered",[]{return fake_follow::b_entered.load();});
 m.def("state",[](int v){fake_follow::state=v;});
 m.def("query_rc",[](int v){fake_follow::query_rc=v;});
 m.def("joint",[](double v){fake_follow::joint=v;});
 m.def("push_enabled",[](bool v){std::lock_guard<std::mutex>lock(fake_follow::periodic_push_mutex);fake_follow::push_enabled=v;});
 m.def("push_enabled_b",[](bool v){std::lock_guard<std::mutex>lock(fake_follow::periodic_push_mutex);fake_follow::push_enabled_b=v;});
 m.def("push_on_observation_read",[]{fake_follow::push_on_observation_read=true;});
 m.def("interleaved_pushes",[]{return fake_follow::interleaved_pushes.load();});
 m.def("next_push_time_offset",[](double v){fake_follow::next_push_time_offset=v;});
 m.def("counts",[]{return std::vector<int>{fake_follow::state_queries_a.load(),fake_follow::state_queries_b.load(),fake_follow::sends.load(),fake_follow::stops.load()};});
}
