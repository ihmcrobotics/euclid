package us.ihmc.euclid.orientation;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation2DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Vector2D;

/**
 * A {@code Orientation2D} represents an orientation in the XY-plane, i.e. the yaw angle about the
 * z-axis.
 */
public class Orientation2D implements Orientation2DBasics, GeometryObject<Orientation2D>
{
   /** The angle in radians about the z-axis. */
   private double yaw = 0.0;

   /** Vector used to transform {@code this} in {@link #applyTransform(Transform)}. */
   private final Vector2D xVector = new Vector2D(Axis2D.X);

   /**
    * Creates a new orientation 2D initialized with its yaw angle to zero.
    */
   public Orientation2D()
   {
      setToZero();
   }

   /**
    * Creates a new orientation 2D and initializes its yaw angle to the given {@code yaw}.
    *
    * @param yaw the yaw angle used to initialize this.
    */
   public Orientation2D(double yaw)
   {
      setYaw(yaw);
   }

   /**
    * Creates a new orientation 2D and initializes it to {@code other}.
    *
    * @param other the other orientation 2D used to initialize this. Not modified.
    */
   public Orientation2D(Orientation2DReadOnly other)
   {
      set(other);
   }

   /**
    * Sets this orientation 2D to the {@code other} orientation 2D.
    *
    * @param other the other orientation 2D. Not modified.
    */
   @Override
   public void set(Orientation2DReadOnly other)
   {
      Orientation2DBasics.super.set(other);
   }

   /**
    * Sets this orientation 2D to the {@code other} orientation 2D.
    *
    * @param other the other orientation 2D. Not modified.
    */
   @Override
   public void set(Orientation2D other)
   {
      Orientation2DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setYaw(double yaw)
   {
      this.yaw = EuclidCoreTools.trimAngleMinusPiToPi(yaw);
   }

   /** {@inheritDoc} */
   @Override
   public double getYaw()
   {
      return yaw;
   }

   /** {@inheritDoc} */
   @Override
   public void applyTransform(Transform transform)
   {
      xVector.set(Axis2D.X);
      transform.transform(xVector);
      double deltaYaw = EuclidCoreTools.atan2(xVector.getY(), xVector.getX());

      if (Double.isNaN(deltaYaw) || Double.isInfinite(deltaYaw))
         deltaYaw = 0.0;

      add(deltaYaw);
   }

   /** {@inheritDoc} */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      xVector.set(Axis2D.X);
      transform.inverseTransform(xVector);
      double deltaYaw = EuclidCoreTools.atan2(xVector.getY(), xVector.getX());

      if (Double.isNaN(deltaYaw) || Double.isInfinite(deltaYaw))
         deltaYaw = 0.0;

      add(deltaYaw);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Orientation2DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Orientation2DReadOnly)
         return equals((Orientation2DReadOnly) object);
      else
         return false;
   }

   /**
    * Tests if the yaw angle of this orientation is equal to an {@code epsilon} to the yaw of
    * {@code other}.
    * <p>
    * Note that this method performs number comparison and not an angle comparison, such that:
    * -<i>pi</i> &ne; <i>pi</i>.
    * </p>
    *
    * @param other   the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two orientations are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Orientation2D other, double epsilon)
   {
      return Orientation2DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two orientations are geometrically
    * similar, i.e. the difference in yaw of {@code this} and {@code other} is less than or equal to
    * {@code epsilon}.
    *
    * @param other   the orientation to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two orientations represent the same geometry, {@code false}
    *         otherwise.
    */
   @Override
   public boolean geometricallyEquals(Orientation2D other, double epsilon)
   {
      return Orientation2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this orientation 2D as follows:<br>
    * (0.123 )
    *
    * @return the {@code String} representing this orientation 2D.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getOrientation2DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of the angle of this orientation 2D.
    *
    * @return the hash code value for this orientation 2D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(yaw);
   }
}
