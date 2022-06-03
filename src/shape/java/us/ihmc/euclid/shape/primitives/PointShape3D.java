package us.ihmc.euclid.shape.primitives;

import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Implementation of a point shape 3D.
 * <p>
 * A point shape 3D is represented by its position.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class PointShape3D implements PointShape3DBasics
{
   private double x, y, z;

   /**
    * Creates a new point shape located at the origin.
    */
   public PointShape3D()
   {
      setToZero();
   }

   /**
    * Creates a new point shape and initializes its position.
    *
    * @param x the value for the x-coordinate of this shape.
    * @param y the value for the y-coordinate of this shape.
    * @param z the value for the z-coordinate of this shape.
    */
   public PointShape3D(double x, double y, double z)
   {
      set(x, y, z);
   }

   /**
    * Creates a new point shape and initializes its position.
    *
    * @param tuple3DReadOnly the position of the point shape. Not modified.
    */
   public PointShape3D(Tuple3DReadOnly tuple3DReadOnly)
   {
      set(tuple3DReadOnly);
   }

   /** {@inheritDoc} */
   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   /** {@inheritDoc} */
   @Override
   public void setZ(double z)
   {
      this.z = z;
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return x;
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return y;
   }

   /** {@inheritDoc} */
   @Override
   public double getZ()
   {
      return z;
   }

   @Override
   public PointShape3D copy()
   {
      return new PointShape3D(this);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Tuple3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PointShape3DReadOnly)
         return PointShape3DBasics.super.equals((PointShape3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this point shape 3D.
    *
    * @return the hash code value for this point shape 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(x, y, z);
   }

   /**
    * Provides a {@code String} representation of this point shape 3D as follows:
    *
    * <pre>
    * Point shape 3D: (-0.362, -0.617,  0.066 )
    * </pre>
    *
    * @return the {@code String} representing this point shape 3D.
    */
   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getPointShape3DString(this);
   }
}
