#include "Processor.h"
#include <cassert>
#include <cstring>

using namespace std;
using namespace ramulator;

Processor::Processor(const Config& configs,
    vector<const char*> trace_list,
    function<bool(Request)> send_memory,
    MemoryBase& memory)
    : ipcs(trace_list.size(), -1),
    early_exit(configs.is_early_exit()),
    no_core_caches(!configs.has_core_caches()),
    no_shared_cache(!configs.has_l3_cache()),
    cachesys(new CacheSystem(configs, send_memory)),
    llc(l3_size, l3_assoc, l3_blocksz,
         mshr_per_bank * trace_list.size(),
         Cache::Level::L3, cachesys) {

  assert(cachesys != nullptr);
  int tracenum = trace_list.size();
  assert(tracenum > 0);
  printf("tracenum: %d\n", tracenum);
  for (int i = 0 ; i < tracenum ; ++i) {
    printf("trace_list[%d]: %s\n", i, trace_list[i]);
  }
  if (no_shared_cache) {
    for (int i = 0 ; i < tracenum ; ++i) {
      cores.emplace_back(new Core(
          configs, i, trace_list[i], send_memory, nullptr,
          cachesys, memory));
    }
  } else {
    for (int i = 0 ; i < tracenum ; ++i) {
      cores.emplace_back(new Core(configs, i, trace_list[i],
          std::bind(&Cache::send, &llc, std::placeholders::_1),
          &llc, cachesys, memory));
    }
  }
  for (int i = 0 ; i < tracenum ; ++i) {
    cores[i]->callback = std::bind(&Processor::receive, this,
        placeholders::_1);
  }

  // regStats
  cpu_cycles.name("cpu_cycles")
            .desc("cpu cycle number")
            .precision(0)
            ;
  cpu_cycles = 0;
}

void Processor::tick() {
  cpu_cycles++;

  if((int(cpu_cycles.value()) % 50000000) == 0)
      printf("CPU heartbeat, cycles: %d \n", (int(cpu_cycles.value())));

  if (!(no_core_caches && no_shared_cache)) {
    cachesys->tick();
  }
  for (unsigned int i = 0 ; i < cores.size() ; ++i) {
    Core* core = cores[i].get();
    core->tick();
  }
}

void Processor::receive(Request& req) {
  if (!no_shared_cache) {
    llc.callback(req);
  } else if (!cores[0]->no_core_caches) {
    // Assume all cores have caches or don't have caches
    // at the same time.
    for (unsigned int i = 0 ; i < cores.size() ; ++i) {
      Core* core = cores[i].get();
      core->caches[0]->callback(req);
    }
  }
  for (unsigned int i = 0 ; i < cores.size() ; ++i) {
    Core* core = cores[i].get();
    core->receive(req);
  }
}

bool Processor::finished() {
  if (early_exit) {
    for (unsigned int i = 0 ; i < cores.size(); ++i) {
      if (cores[i]->finished()) {
        for (unsigned int j = 0 ; j < cores.size() ; ++j) {
          ipc += cores[j]->calc_ipc();
        }
        return true;
      }
    }
    return false;
  } else {
    for (unsigned int i = 0 ; i < cores.size(); ++i) {
      if (!cores[i]->finished()) {
        return false;
      }
      if (ipcs[i] < 0) {
        ipcs[i] = cores[i]->calc_ipc();
        ipc += ipcs[i];
      }
    }
    return true;
  }
}

bool Processor::has_reached_limit() {
  for (unsigned int i = 0 ; i < cores.size() ; ++i) {
    if (!cores[i]->has_reached_limit()) {
      return false;
    }
  }
  return true;
}

long Processor::get_insts() {
    long insts_total = 0;
    for (unsigned int i = 0 ; i < cores.size(); i++) {
        insts_total += cores[i]->get_insts();
    }

    return insts_total;
}

void Processor::reset_stats() {
    for (unsigned int i = 0 ; i < cores.size(); i++) {
        cores[i]->reset_stats();
    }

    ipc = 0;

    for (unsigned int i = 0; i < ipcs.size(); i++)
        ipcs[i] = -1;
}

Core::Core(const Config& configs, int coreid,
    const char* trace_fname, function<bool(Request)> send_next,
    Cache* llc, std::shared_ptr<CacheSystem> cachesys, MemoryBase& memory)
    : id(coreid), no_core_caches(!configs.has_core_caches()),
    no_shared_cache(!configs.has_l3_cache()),
    llc(llc), trace(trace_fname), memory(memory)
{
  // set expected limit instruction for calculating weighted speedup
  expected_limit_insts = configs.get_expected_limit_insts();
  trace.expected_limit_insts = expected_limit_insts;

  // Build cache hierarchy
  if (no_core_caches) {
    send = send_next;
  } else {
    // L2 caches[0]
    caches.emplace_back(new Cache(
        l2_size, l2_assoc, l2_blocksz, l2_mshr_num,
        Cache::Level::L2, cachesys));
    // L1 caches[1]
    caches.emplace_back(new Cache(
        l1_size, l1_assoc, l1_blocksz, l1_mshr_num,
        Cache::Level::L1, cachesys));
    send = bind(&Cache::send, caches[1].get(), placeholders::_1);
    if (llc != nullptr) {
      caches[0]->concatlower(llc);
    }
    caches[1]->concatlower(caches[0].get());

    first_level_cache = caches[1].get();
  }
  if (no_core_caches) {
    if(configs["trace_type"] == "CPU")
      more_reqs = trace.get_filtered_request(
        bubble_cnt, req_addr, req_type);
    else if (configs["trace_type"] == "DATADEP" ) {
      cputrace = false;
      more_reqs = trace.get_dependence_request(
        bubble_cnt, req_addr, req_type, seq_number, dep_addr);

    }
    req_addr = memory.page_allocator(req_addr, id);
  } else {
    more_reqs = trace.get_unfiltered_request(
        bubble_cnt, req_addr, req_type);
    req_addr = memory.page_allocator(req_addr, id);
  }


  // regStats
  record_cycs.name("record_cycs_core_" + to_string(id))
             .desc("Record cycle number for calculating weighted speedup. (Only valid when expected limit instruction number is non zero in config file.)")
             .precision(0)
             ;

  record_insts.name("record_insts_core_" + to_string(id))
              .desc("Retired instruction number when record cycle number. (Only valid when expected limit instruction number is non zero in config file.)")
              .precision(0)
              ;

  memory_access_cycles.name("memory_access_cycles_core_" + to_string(id))
                      .desc("memory access cycles in memory time domain")
                      .precision(0)
                      ;
  memory_access_cycles = 0;
  cpu_inst.name("cpu_instructions_core_" + to_string(id))
          .desc("cpu instruction number")
          .precision(0)
          ;
  cpu_inst = 0;
}


double Core::calc_ipc()
{
    printf("[%d]retired: %ld, clk, %ld\n", id, retired, clk);
    return (double) retired / clk;
}
bool Core::get_ready(long addr)
{
  for(std::list<long>::iterator it = pendingreads->begin(); it!=pendingreads->end();++it) {
    if((*it)==addr) return false;
  }
  return true;
}
void Core::tick()
{
    clk++;
    if(first_level_cache != nullptr)
        first_level_cache->tick();

    retired += window.retire();

    if (expected_limit_insts == 0 && !more_reqs) return;

    // bubbles (non-memory operations)
    int inserted = 0;
    while (bubble_cnt > 0) {
        if (inserted == window.ipc) return;
        if (window.is_full()) return;
        window.insert(true, -1);
        inserted++;
        bubble_cnt--;
        cpu_inst++;
        if (long(cpu_inst.value()) == expected_limit_insts && !reached_limit) {
          record_cycs = clk;
          record_insts = long(cpu_inst.value());
          memory.record_core(id);
          reached_limit = true;
        }
    }
    //printf("checking: %li ready: %d\n",dep_addr,get_ready(dep_addr));
    if(dep_addr!=-1 && !get_ready(dep_addr)) {
      //printf("added bubble for: %li %d - %li\n",req_addr,req_type,dep_addr);
      bubble_cnt++;
      return;
    }

    if (req_type == Request::Type::READ) {
        // read request
        if (inserted == window.ipc) return;
        if (window.is_full()) return;

        Request req(req_addr, req_type, callback, id);
        if (!send(req)) return;
        //printf("head: %d, seq_number: %d",window.get_head(),seq_number);
        window.insert(false, req_addr);
        //printf("inserting: %li\n",req_addr);
        pendingreads->push_back(req_addr);
        cpu_inst++;
    }
    else {
        // write request
        assert(req_type == Request::Type::WRITE);
        Request req(req_addr, req_type, callback, id);
        if (!send(req)) return;
        cpu_inst++;
    }
    if (long(cpu_inst.value()) == expected_limit_insts && !reached_limit) {
      record_cycs = clk;
      record_insts = long(cpu_inst.value());
      memory.record_core(id);
      reached_limit = true;
    }

    //printf("req: %d addr: %li bbl: %li seq_number: %d dep_addr: %li\n",req_type, req_addr, bubble_cnt, seq_number, dep_addr);
    if (no_core_caches) {
      if(cputrace)
        more_reqs = trace.get_filtered_request(
          bubble_cnt, req_addr, req_type);
      else
        more_reqs = trace.get_dependence_request(
        bubble_cnt, req_addr, req_type, seq_number, dep_addr);
      if (req_addr != -1) {
        req_addr = memory.page_allocator(req_addr, id);
      }
    } else {
      more_reqs = trace.get_unfiltered_request(
          bubble_cnt, req_addr, req_type);
      if (req_addr != -1) {
        req_addr = memory.page_allocator(req_addr, id);
      }
    }
    if (!more_reqs) {
      if (!reached_limit) { // if the length of this trace is shorter than expected length, then record it when the whole trace finishes, and set reached_limit to true.
        // Hasan: overriding this behavior. We start the trace from the
        // beginning until the requested amount of instructions are
        // simulated. This should never be reached now.
        assert((expected_limit_insts == 0) && "Shouldn't be reached when expected_limit_insts > 0 since we start over the trace");
        record_cycs = clk;
        record_insts = long(cpu_inst.value());
        memory.record_core(id);
        reached_limit = true;
      }
    }
}

bool Core::finished()
{
    return !more_reqs && window.is_empty();
}

bool Core::has_reached_limit() {
  return reached_limit;
}

long Core::get_insts() {
    return long(cpu_inst.value());
}

void Core::receive(Request& req)
{
    window.set_ready(req.addr, ~(l1_blocksz - 1l));
    if (req.arrive != -1 && req.depart > last) {
      memory_access_cycles += (req.depart - max(last, req.arrive));
      last = req.depart;
    }
    //printf("removing %li\n",req.addr);
    pendingreads->remove(req.addr);
}

void Core::reset_stats() {
    clk = 0;
    retired = 0;
    cpu_inst = 0;
}

bool Window::is_full()
{
    return load == depth;
}

bool Window::is_empty()
{
    return load == 0;
}


void Window::insert(bool ready, long addr)
{
    assert(load <= depth);

    ready_list.at(head) = ready;
    addr_list.at(head) = addr;

    head = (head + 1) % depth;
    load++;
}


long Window::retire()
{
    assert(load <= depth);

    if (load == 0) return 0;

    int retired = 0;
    while (load > 0 && retired < ipc) {
        if (!ready_list.at(tail))
            break;

        tail = (tail + 1) % depth;
        load--;
        retired++;
    }

    return retired;
}


void Window::set_ready(long addr, int mask)
{
    if (load == 0) return;

    for (int i = 0; i < load; i++) {
        int index = (tail + i) % depth;
        if ((addr_list.at(index) & mask) != (addr & mask))
            continue;
        ready_list.at(index) = true;
    }
}

Trace::Trace(const char* trace_fname) : file(trace_fname), trace_name(trace_fname)
{
    if (!file.good()) {
        std::cerr << "Bad trace file: " << trace_fname << std::endl;
        exit(1);
    }
}

bool Trace::get_unfiltered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type)
{
    string line;
    getline(file, line);
    if (file.eof()) {
      file.clear();
      file.seekg(0, file.beg);
      getline(file, line);
      //return false;
    }
    size_t pos, end;
    bubble_cnt = std::stoul(line, &pos, 10);
    pos = line.find_first_not_of(' ', pos+1);
    req_addr = std::stoul(line.substr(pos), &end, 0);

    pos = line.find_first_not_of(' ', pos+end);

    if (pos == string::npos || line.substr(pos)[0] == 'R')
        req_type = Request::Type::READ;
    else if (line.substr(pos)[0] == 'W')
        req_type = Request::Type::WRITE;
    else assert(false);
    return true;
}

bool Trace::get_filtered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type)
{
    static bool has_write = false;
    static long write_addr;
    static int line_num = 0;
    if (has_write){
        bubble_cnt = 0;
        req_addr = write_addr;
        req_type = Request::Type::WRITE;
        has_write = false;
        return true;
    }
    string line;
    getline(file, line);
    line_num ++;
    if (file.eof() || line.size() == 0) {
        file.clear();
        file.seekg(0, file.beg);
        line_num = 0;

        if(expected_limit_insts == 0) {
            has_write = false;
            return false;
        }
        else { // starting over the input trace file
            getline(file, line);
            line_num++;
        }
    }

    size_t pos, end;
    bubble_cnt = std::stoul(line, &pos, 10);

    pos = line.find_first_not_of(' ', pos+1);
    req_addr = stoul(line.substr(pos), &end, 0);
    req_type = Request::Type::READ;

    pos = line.find_first_not_of(' ', pos+end);
    if (pos != string::npos){
        has_write = true;
        write_addr = stoul(line.substr(pos), NULL, 0);
    }
    return true;
}
//for data dependency traces
bool Trace::get_dependence_request(long& bubble_cnt, long& req_addr, Request::Type& req_type, int& seq_number, long& dep_addr)
{
    static bool has_write = false;
    static int line_num = 0;
    string line;
    int dep_number = -1;
    while(true){
      getline(file, line);
      //printf("%s\n",line.c_str());
      line_num ++;
      if (file.eof() || line.size() == 0) {
          file.clear();
          file.seekg(0, file.beg);
          line_num = 0;

          if(expected_limit_insts == 0) {
              has_write = false;
              return false;
          }
          else { // starting over the input trace file
              getline(file, line);
              line_num++;
          }
      }
      char *token = strtok(strdup(line.c_str()), " ");
      //printf("token seq_no: %s\n",token);
      seq_number = stoi(token);
      readlist[seq_number]=false;
      readlist_addr[seq_number]=-1;
      token = strtok(NULL," ");
      //printf("token type: %s\n",token);
      bool iscomp = false;
      bool dependent = false;
      readlist[seq_number] = false;
      readlist_addr[seq_number] = 0;
      if(std::strcmp(token,"READ") == 0) {
        req_type = Request::Type::READ;
        readlist[seq_number] = true;
      } else if(std::strcmp(token,"WRITE") == 0) {
        req_type = Request::Type::WRITE;
      } else if(std::strcmp(token,"COMP") == 0 ||
          std::strcmp(token,"READ[C]") == 0    || //TODO: maybe handle these different
          std::strcmp(token,"WRITE[C]") == 0   ){ //^
        iscomp = true;
        bubble_cnt++;
        //printf("Bubble count: %li\n",bubble_cnt);
        if(std::strcmp(token,"COMP")!=0)
          token = strtok(NULL," ");
      } else {
        printf("Bad trace file. found: %s\n",token);
        return false;
      }
      if(!iscomp) {
        token = strtok(NULL," ");
        //printf("token addr: %s\n",token);
        req_addr = stoul(token,NULL, 0);
        if(readlist[seq_number]) {
          readlist_addr[seq_number]=req_addr;
          //printf("adding: seq: %d, addr: %li\n", seq_number, req_addr);
        }
      }
      //dependencies
      token = strtok(NULL," ");
      if (token!=NULL){
        //int dep_number=-1;
        //dep_addr = -1;
        dependent = true;
        token = strtok(NULL," ");
        while(token!=NULL) {
          //printf("token deps: %s\n",token);
          int current = atoi(token);
          if(!readlist[current])  {
            token = strtok(NULL, " ");
            continue;
          }
          if(dep_number==-1)
            dep_number = current;
          else { //compare the old and the new dependencies choose the closer one
            int n_dif = seq_number - current;
            int dif = seq_number - dep_number;
            if(dif<0 && n_dif<0)
              dep_number = (dif>n_dif) ? current : dep_number;
            else if(dif<0 && n_dif>0)
              dep_number = current;
            else if(dif>0 && n_dif>0)
              dep_number = (dif>n_dif) ? current : dep_number;
          }
          token = strtok(NULL," ");
        }
        dep_addr = (dep_number != -1) ? readlist_addr[dep_number] : -1;
        //if(dep_number!=-1)
        //printf("dep_number: %d, dep_addr: %li\n",dep_number, dep_addr);
      }
      if(!iscomp)  //need to count bubbles until a memory instruction.
      {
        dep_number = -1;
        dep_addr = -1;
        return true;
      }
    }
    return true; //not return true actually because you have some comp instructions left this is probably wrong.
}

bool Trace::get_dramtrace_request(long& req_addr, Request::Type& req_type)
{
    string line;
    getline(file, line);
    if (file.eof()) {
        return false;
    }
    size_t pos;
    req_addr = std::stoul(line, &pos, 16);

    pos = line.find_first_not_of(' ', pos+1);

    if (pos == string::npos || line.substr(pos)[0] == 'R')
        req_type = Request::Type::READ;
    else if (line.substr(pos)[0] == 'W')
        req_type = Request::Type::WRITE;
    else assert(false);
    return true;
}
