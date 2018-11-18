package us.ihmc.euclid.shape;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.shape.interfaces.Ramp3DBasics;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * {@code Ramp3D} represents a 3D shape with a triangular section in the XZ-plane.
 * <p>
 * Shape description:
 * <ul>
 * <li>The slope face starts from {@code x=0.0}, {@code z=0.0} to end at {@code x=size.getX()},
 * {@code z=size.getZ()}.
 * <li>The bottom face is horizontal (XY-plane) at {@code z=0.0}.
 * <li>The rear face is vertical (YZ-plane) at {@code x=size.getX()}.
 * <li>The left face is vertical (XZ-plane) at {@code y=-size.getY()/2.0}.
 * <li>The right face is vertical (XZ-plane) at {@code y=size.getY()/2.0}.
 * </ul>
 * </p>
 */
public class Ramp3D extends Shape3D implements Ramp3DBasics, GeometryObject<Ramp3D>
{
   /** Size of this ramp's bounding box. */
   private final Vector3D size = new Vector3D()
   {
      @Override
      public void setX(double x)
      {
         if (x < 0.0)
            throw new IllegalArgumentException("The x-size of a Ramp3D cannot be negative: " + x);
         super.setX(x);
         updateRamp();
      }

      @Override
      public void setY(double y)
      {
         if (y < 0.0)
            throw new IllegalArgumentException("The y-size of a Ramp3D cannot be negative: " + y);
         super.setY(y);
      }

      @Override
      public void setZ(double z)
      {
         if (z < 0.0)
            throw new IllegalArgumentException("The z-size of a Ramp3D cannot be negative: " + z);
         super.setZ(z);
         updateRamp();
      }
   };

   /** Length of the slope face of this ramp. */
   private double rampLength;
   /**
    * Positive angle in [0, <i>pi</i>] representing the angle formed by the bottom face and the slope
    * face.
    */
   private double angleOfRampIncline;

   /**
    * Creates a new ramp 3D and initializes its length, width, and height to {@code 1.0}.
    */
   public Ramp3D()
   {
      this(1.0, 1.0, 1.0);
   }

   /**
    * Creates a new ramp 3D identical to {@code other}.
    *
    * @param other the other ramp to copy. Not modified.
    */
   public Ramp3D(Ramp3D other)
   {
      set(other);
   }

   /**
    * Creates a new ramp 3D and initializes its size.
    *
    * @param length the size of this ramp along the x-axis.
    * @param width the size of this ramp along the y-axis.
    * @param height the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code length}, {@code width}, or {@code height} is
    *            negative.
    */
   public Ramp3D(double length, double width, double height)
   {
      setSize(length, width, height);
   }

   /**
    * Creates a new ramp 3D and initializes its pose and size.
    *
    * @param pose the position and orientation for this ramp. Not modified.
    * @param length the size of this ramp along the x-axis.
    * @param width the size of this ramp along the y-axis.
    * @param height the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code length}, {@code width}, or {@code height} is
    *            negative.
    */
   public Ramp3D(RigidBodyTransformReadOnly pose, double length, double width, double height)
   {
      set(pose, length, width, height);
   }

   /**
    * Creates a new ramp 3D and initializes its pose and size.
    *
    * @param pose the position and orientation for this ramp. Not modified.
    * @param length the size of this ramp along the x-axis.
    * @param width the size of this ramp along the y-axis.
    * @param height the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code length}, {@code width}, or {@code height} is
    *            negative.
    */
   public Ramp3D(Pose3DReadOnly pose, double length, double width, double height)
   {
      set(pose, length, width, height);
   }

   /**
    * Copies the {@code other} ramp data into {@code this}.
    *
    * @param other the other ramp to copy. Not modified.
    */
   @Override
   public void set(Ramp3D other)
   {
      Ramp3DBasics.super.set(other);
   }

   private void updateRamp()
   {
      rampLength = EuclidShapeTools.computeRamp3DLength(getSize());
      angleOfRampIncline = EuclidShapeTools.computeRanp3DIncline(getSize());
   }

   @Override
   public Vector3DBasics getSize()
   {
      return size;
   }

   /**
    * Gets the length of this ramp's slope part.
    * <p>
    * Note that this is different than {@link #getSizeX()}. The returned value is equal to:
    * &radic;(this.length<sup>2</sup> + this.height<sup>2</sup>)
    * </p>
    *
    * @return the length of the slope.
    */
   @Override
   public double getRampLength()
   {
      return rampLength;
   }

   /**
    * Gets the angle formed by the slope and the bottom face.
    * <p>
    * The angle is positive and in [0, <i>pi</i>].
    * </p>
    *
    * @return the slope angle.
    */
   @Override
   public double getRampIncline()
   {
      return angleOfRampIncline;
   }

   /** {@inheritDoc} */
   @Override
   protected double evaluateQuery(Point3DReadOnly query, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      return EuclidShapeTools.evaluatePoint3DWithRamp3D(query, closestPointToPack, normalToPack, getSize());
   }

   /** {@inheritDoc} */
   @Override
   protected boolean isInsideEpsilonShapeFrame(Point3DReadOnly query, double epsilon)
   {
      return EuclidShapeTools.isPoint3DInsideRamp3D(query, epsilon, getSize());
   }

   /**
    * Tests separately and on a per component basis if the pose and the size of this ramp and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    *
    * @param other the other ramp which pose and size is to be compared against this ramp pose and
    *           size. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two ramps are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Ramp3D other, double epsilon)
   {
      return Ramp3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two ramps are geometrically similar,
    * i.e. the difference between their size are less than or equal to {@code epsilon} and their poses
    * are geometrically similar given {@code epsilon}.
    *
    * @param other the ramp to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the ramps represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Ramp3D other, double epsilon)
   {
      return Ramp3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this ramp 3D as follows:<br>
    * Ramp 3D: size = (length, width, height), pose = <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this box 3D.
    */
   @Override
   public String toString()
   {
      return "Ramp 3D: size = " + size + ", + pose =\n" + getPoseString();
   }
}
