package us.ihmc.euclid.shape.collision.shapeModifier.interfaces;

import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;

public interface GJKShape3DModifier
{
   SupportingVertexHolder toSupportingVertexHolder(Shape3DReadOnly shape);
}
