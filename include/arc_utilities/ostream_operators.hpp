#ifndef LBV_OSTREAM_OPERATORS
#define LBV_OSTREAM_OPERATORS

#include <moveit/collision_detection/collision_common.h>
#include <ostream>
#include <typeinfo>

static std::ostream &operator<<(std::ostream &out, collision_detection::CollisionResult const &cr)
{
  if (not cr.collision)
  {
    out << "no collision";
  } else
  {
    if (cr.contacts.empty())
    {
      out << "collision found but no contacts reported";
    } else
    {
      auto const &[name, contacts] = *cr.contacts.cbegin();
      if (not contacts.empty())
      {
        auto const contact = contacts.front();
        out << "collision between " << contact.body_name_1 << " and " << contact.body_name_2;
        bool multiple_contacts = cr.contacts.size() > 1 or contacts.size() > 1;
        if (multiple_contacts)
        {
          out << " among others...\n";
        }
      }
    }
  }
  return out;
}

template<typename T>
static std::ostream &operator<<(std::ostream &out, std::vector<T> const &vec)
{
  for (auto const &val : vec)
  {
    out << val;
    if (typeid(T) != typeid(std::string))
    {
      out << " ";
    } else
    {
      out << "\n";
    }
  }
  return out;
}

#endif
