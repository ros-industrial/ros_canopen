#ifndef SOCKETCAN_INTERFACE_SERIALIZABLE_MAP_H
#define SOCKETCAN_INTERFACE_SERIALIZABLE_MAP_H

#include <map>
#include <sstream>

template<class Key, class Value>
class serializable_map : public std::map<Key, Value> {
private:
  size_t offset;

  template<class T>
  void write(std::stringstream &ss, T &t) const {
    ss.write((char*)(&t), sizeof(t));
  }

  void write(std::stringstream &ss, std::string &str) const {
    size_t size = str.size();
    ss.write((char*)(&size), sizeof(size));
    ss.write((char*)(str.data()), str.length());
  }

  template<class T>
  void read(const std::string &s, T &t) {
    t = (T)(*(s.data() + offset));
    offset += sizeof(T);
  }

  void read(const std::string &s, std::string &str) {
    size_t size = (int)(*(s.data() + offset));
    offset += sizeof(size_t);
    std::string str2(s.data() + offset, s.data() + offset + size);
    str = str2;
    offset += size;
  }
public:
  std::string serialize() const {
    std::stringstream ss;
    for (auto &i : (*this)) {
      Key str = i.first;
      Value value = i.second;
      write(ss, str);
      write(ss, value);
    }
    return ss.str();
  }
  void deserialize(const std::string &s) {
    offset = 0;
    while (offset < s.size()) {
      Key key;
      Value value;
      read(s, key);
      read(s, value);
      (*this)[key] = value;
    }
  }
};

#endif /* SOCKETCAN_INTERFACE_SERIALIZABLE_MAP_H */
