#pragma once

#include <stdexcept>


class collision_object_creation_error : public std::runtime_error {
  public:
    collision_object_creation_error(const std::string &msg = "Collision Object creation error")
        : std::runtime_error(msg) {}
};


class json_field_error : public std::runtime_error {
  public:
    json_field_error(const std::string &msg = "Json field error") : std::runtime_error(msg) {}
};


class planning_error : public std::runtime_error {
  public:
    planning_error(const std::string &msg = "Planning error") : std::runtime_error(msg) {}
};