#ifndef TERRAIN_HPP_
#define TERRAIN_HPP_

#include <string>

namespace shared {

    class Terrain {
        public:
            Terrain(const std::string name, 
                    const double traversalCost, 
                    const double velocityDamping,
                    bool traversable);

            ~Terrain() = default;

            const std::string getName() const;

            const double getTraversalCost() const;

            const double getVelocityDamping() const;

            const bool isTraversable() const;            

        private:
            const std::string name_;
            const double traversalCost_;
            const double velocityDamping_;
            const bool traversable_;
     };

}

#endif
