package us.ihmc.euclid.shape;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.shape.interfaces.Capsule3DBasics;
import us.ihmc.euclid.shape.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Capsule3D implements Capsule3DBasics, GeometryObject<Capsule3D>
{
   private final Shape3DPose pose = new Shape3DPose();

   private double radius;
   private double length;
   private double halfLength;

   private final Point3DReadOnly topCenter = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> getHalfLength() * getAxis().getX() + getPositionX(),
                                                                                          () -> getHalfLength() * getAxis().getY() + getPositionY(),
                                                                                          () -> getHalfLength() * getAxis().getZ() + getPositionZ());
   private final Point3DReadOnly bottomCenter = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> -getHalfLength() * getAxis().getX() + getPositionX(),
                                                                                             () -> -getHalfLength() * getAxis().getY() + getPositionY(),
                                                                                             () -> -getHalfLength() * getAxis().getZ() + getPositionZ());

   public Capsule3D()
   {
      this(1.0, 0.5);
   }

   public Capsule3D(Capsule3DReadOnly other)
   {
      set(other);
   }

   public Capsule3D(double length, double radius)
   {
      setSize(length, radius);
   }

   public Capsule3D(RigidBodyTransformReadOnly pose, double length, double radius)
   {
      set(pose, length, radius);
   }

   public Capsule3D(Pose3DReadOnly pose, double length, double radius)
   {
      set(pose, length, radius);
   }

   @Override
   public void set(Capsule3D other)
   {
      getPose().set(other.getPose());
      setRadius(other.getRadius());
      setLength(other.getLength());
   }

   @Override
   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   @Override
   public void setLength(double length)
   {
      this.length = length;
      halfLength = 0.5 * length;
   }

   @Override
   public double getRadius()
   {
      return radius;
   }

   @Override
   public double getLength()
   {
      return length;
   }

   @Override
   public double getHalfLength()
   {
      return halfLength;
   }

   @Override
   public Shape3DPose getPose()
   {
      return pose;
   }

   @Override
   public Point3DReadOnly getTopCenter()
   {
      return topCenter;
   }

   @Override
   public Point3DReadOnly getBottomCenter()
   {
      return bottomCenter;
   }

   @Override
   public boolean epsilonEquals(Capsule3D other, double epsilon)
   {
      return Capsule3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(Capsule3D other, double epsilon)
   {
      return Capsule3DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getCapsule3DString(this);
   }
}
