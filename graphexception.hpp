#pragma once
#include <string>
#include <assert.h>

class GraphException
{
public:
    GraphException(const std::string& _err)
        : err{_err}{}
    const std::string& what() const {return err;}
private:
    std::string err;
};
