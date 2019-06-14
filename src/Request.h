#ifndef __REQUEST_H
#define __REQUEST_H

#include <vector>
#include <functional>

using namespace std;

namespace ramulator
{

class Request
{
public:
    bool is_first_command;
    bool is_random_read;
    long addr;
    // long addr_row;
    vector<int> addr_vec;
    // specify which core this request sent from, for virtual address translation
    int coreid;

    enum class Type
    {
        READ,
        WRITE,
        REFRESH,
        POWERDOWN,
        SELFREFRESH,
        EXTENSION,
        RANDOM,
        MAX
    } type;

    long arrive = -1;
    long depart;
    function<void(Request&)> callback; // call back with more info

    Request(long addr, Type type, int coreid = 0)
        : is_first_command(true), is_random_read(false), addr(addr), coreid(coreid), type(type),
      callback([](Request& req){}) {}

    Request(long addr, Type type, function<void(Request&)> callback, int coreid = 0)
        : is_first_command(true), is_random_read(false), addr(addr), coreid(coreid), type(type), callback(callback) {}

    Request(vector<int>& addr_vec, Type type, function<void(Request&)> callback, int coreid = 0)
        : is_first_command(true), is_random_read(false), addr_vec(addr_vec), coreid(coreid), type(type), callback(callback) {}
  //  Request (vector<int>& addr_vec, Type type, function<void(Request&)> callback, int coreid = 0, bool is_random_read = false )
  //      :  is_first_command(true), is_random_read(is_random_read), addr_vec(addr_vec), coreid(coreid), type(type), callback(callback) {}
    Request()
        : is_first_command(true), is_random_read(false), coreid(0) {}
};

} /*namespace ramulator*/

#endif /*__REQUEST_H*/
