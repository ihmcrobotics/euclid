package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DReadOnly;

public class FrameLine3D implements FrameLine3DBasics, GeometryObject<FrameLine3D>
{
   /** The line. */
   private final Line3D line = new Line3D();

   private ReferenceFrame referenceFrame;

   private final FixedFramePoint3DBasics point = new FixedFramePoint3DBasics()
   {
      @Override
      public void setX(double x)
      {
         line.getPoint().setX(x);
      }

      @Override
      public void setY(double y)
      {
         line.getPoint().setY(y);
      }

      @Override
      public void setZ(double z)
      {
         line.getPoint().setZ(z);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return line.getPointX();
      }

      @Override
      public double getY()
      {
         return line.getPointY();
      }

      @Override
      public double getZ()
      {
         return line.getPointZ();
      }
   };

   private final FixedFrameVector3DBasics direction = new FixedFrameVector3DBasics()
   {
      @Override
      public void setX(double x)
      {
         line.getDirection().setX(x);
      }

      @Override
      public void setY(double y)
      {
         line.getDirection().setY(y);
      }

      @Override
      public void setZ(double z)
      {
         line.getDirection().setZ(z);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return line.getDirectionX();
      }

      @Override
      public double getY()
      {
         return line.getDirectionY();
      }

      @Override
      public double getZ()
      {
         return line.getDirectionZ();
      }
   };

   public FrameLine3D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   public FrameLine3D(Line3DReadOnly line3D)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), line3D);
   }

   public FrameLine3D(ReferenceFrame referenceFrame, Line3DReadOnly line3D)
   {
      setIncludingFrame(referenceFrame, line3D);
   }

   @Override
   public void set(FrameLine3D other)
   {
      FrameLine3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getPoint()
   {
      return point;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getDirection()
   {
      return direction;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Tests on a per-component basis on the point and vector if this line is equal to {@code other}
    * with the tolerance {@code epsilon}. This method will return {@code false} if the two lines are
    * physically the same but either the point or vector of each line is different. For instance, if
    * {@code this.point == other.point} and {@code this.direction == - other.direction}, the two
    * lines are physically the same but this method returns {@code false}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two lines are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameLine3D other, double epsilon)
   {
      return FrameLine3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two lines are geometrically
    * similar.
    * <p>
    * Two lines are considered geometrically equal is they are collinear, pointing toward the same
    * or opposite direction.
    * </p>
    *
    * @param other the line to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two lines represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   @Override
   public boolean geometricallyEquals(FrameLine3D other, double epsilon)
   {
      return FrameLine3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(FrameLine3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param obj the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((FrameLine3DReadOnly) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Provides a {@code String} representation of this frame line 3D as follows:<br>
    * Line 3D: point = (x, y, z), direction = (x, y, z)-worldFrame
    *
    * @return the {@code String} representing this line 3D.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getLine3DString(this) + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this line 3D.
    *
    * @return the hash code value for this line 3D.
    */
   @Override
   public int hashCode()
   {
      return line.hashCode();
   }
}
