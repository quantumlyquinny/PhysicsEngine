#include "SupportPoint.hpp"
#include "IConvexShape.hpp"
#include <glm/gtx/quaternion.hpp>

glm::vec3 TransformedShape::support(const glm::vec3& worldDir) const {
    const glm::vec3 localDir = glm::transpose(rotation) * worldDir;
    return rotation * shape->getSupport(localDir) + position;
}

TransformedShape TransformedShape::from(const IConvexShape* shape,
                                        const glm::vec3&    position,
                                        const glm::quat&    orientation)
{
    return { shape, position, glm::toMat3(orientation) };
}