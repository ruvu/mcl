#include <random>

#include "./json_utils.hpp"
#include "./landmark.hpp"

int main(int argc, char ** argv)
{
  std::mt19937 gen{std::random_device{}()};
  std::uniform_real_distribution<> dx(-500, 500);
  std::uniform_real_distribution<> dt(0, M_2_PI);

  std::vector<Landmark> landmarks;
  for (int i = 0; i < 500; ++i) {
    tf2::Quaternion q;
    q.setRPY(0, 0, dt(gen));
    landmarks.emplace_back(tf2::Transform{q, tf2::Vector3{dx(gen), dx(gen), 0}});
  }

  json j = landmarks;
  //  std::ofstream file("map");
  std::cout << std::setw(2) << j;

  return EXIT_SUCCESS;
}
