package us.ihmc.euclid.shape;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.shape.interfaces.PointShape3DBasics;
import us.ihmc.euclid.shape.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class PointShape3D implements PointShape3DBasics, GeometryObject<PointShape3D>
{
   private final Shape3DPose pose = new Shape3DPose();

   public PointShape3D()
   {
      setToZero();
   }

   public PointShape3D(Tuple3DReadOnly tuple3DReadOnly)
   {
      set(tuple3DReadOnly);
   }

   @Override
   public void set(PointShape3D other)
   {
      PointShape3DBasics.super.set(other);
   }

   @Override
   public Shape3DPose getPose()
   {
      return pose;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PointShape3DReadOnly)
         return PointShape3DBasics.super.equals((PointShape3DReadOnly) object);
      else
         return false;
   }

   @Override
   public boolean epsilonEquals(PointShape3D other, double epsilon)
   {
      return PointShape3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(PointShape3D other, double epsilon)
   {
      return PointShape3DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public int hashCode()
   {
      return pose.hashCode();
   }

   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getPointShape3DString(this);
   }
}
