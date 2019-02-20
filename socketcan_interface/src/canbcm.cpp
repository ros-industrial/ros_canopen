// Copyright (c) 2016-2019, Mathias Lüdtke, AutonomouStuff
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "socketcan_interface/bcm.hpp"
#include "socketcan_interface/string.hpp"

using namespace can;

#include <iostream>

int main(int argc, char *argv[])
{
  BCMsocket bcm;

  int extra_frames = argc - 4;

  if (extra_frames < 0)
  {
    std::cout << "usage: " << argv[0] << " DEVICE PERIOD HEADER#DATA [DATA*]" << std::endl;
    return 1;
  }

  if (!bcm.init(argv[1]))
  {
    return 2;
  }

  int num_frames = extra_frames + 1;
  Frame *frames = new Frame[num_frames];
  Header header = frames[0] = toframe(argv[3]);

  if (extra_frames > 0)
  {
    for (int i = 0; i < extra_frames; ++i)
    {
      frames[1 + i] = toframe(tostring(header, true) + "#" + argv[4 + i]);
    }
  }
  for (int i = 0; i < num_frames; ++i)
  {
    std::cout << frames[i] << std::endl;
  }
  if (bcm.startTX(boost::chrono::duration<double>(atof(argv[2])), header, num_frames, frames))
  {
    pause();
    return 0;
  }
  return 4;
}
