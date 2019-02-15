package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public interface PointShape3DBasics extends PointShape3DReadOnly, Shape3DBasics, Point3DBasics
{
   @Override
   default boolean containsNaN()
   {
      return PointShape3DReadOnly.super.containsNaN();
   }
}
