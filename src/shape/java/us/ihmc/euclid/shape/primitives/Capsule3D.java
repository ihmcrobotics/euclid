package us.ihmc.euclid.shape.primitives;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Capsule3D implements Capsule3DBasics, GeometryObject<Capsule3D>
{
   private final Point3D position = new Point3D();
   private final Vector3D axis = new Vector3D(Axis.Z);

   private double radius;
   private double length;
   private double halfLength;

   private final Point3DReadOnly topCenter = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> halfLength * axis.getX() + position.getX(),
                                                                                          () -> halfLength * axis.getY() + position.getY(),
                                                                                          () -> halfLength * axis.getZ() + position.getZ());
   private final Point3DReadOnly bottomCenter = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> -halfLength * axis.getX() + position.getX(),
                                                                                             () -> -halfLength * axis.getY() + position.getY(),
                                                                                             () -> -halfLength * axis.getZ() + position.getZ());

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

   public Capsule3D(Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      set(position, axis, length, radius);
   }

   @Override
   public void set(Capsule3D other)
   {
      Capsule3DBasics.super.set(other);
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
   public Point3D getPosition()
   {
      return position;
   }

   @Override
   public Vector3D getAxis()
   {
      return axis;
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
