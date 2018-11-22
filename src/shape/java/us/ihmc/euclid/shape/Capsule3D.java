package us.ihmc.euclid.shape;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.interfaces.Capsule3DBasics;
import us.ihmc.euclid.shape.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Capsule3D implements Capsule3DBasics, GeometryObject<Capsule3D>
{
   private final Shape3DPose pose = new Shape3DPose();
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();

   private double radius;
   private double length;

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
      setPose(other);
      setRadius(other.getRadius());
      setLength(other.getLength());
   }

   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   public void setLength(double length)
   {
      this.length = length;
   }

   public double getRadius()
   {
      return radius;
   }

   public double getLength()
   {
      return length;
   }

   public Vector3DReadOnly getAxis()
   {
      return pose.getZAxis();
   }

   @Override
   public Shape3DPose getPose()
   {
      return pose;
   }

   @Override
   public RotationMatrix getOrientation()
   {
      return pose.getShapeOrientation();
   }

   @Override
   public Point3DBasics getPosition()
   {
      return pose.getShapePosition();
   }

   @Override
   public IntermediateVariableSupplier getIntermediateVariableSupplier()
   {
      return supplier;
   }

   @Override
   public void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier)
   {
      this.supplier = newSupplier;
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
