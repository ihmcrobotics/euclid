package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DReadOnly;

public class FrameLineSegment3D implements FrameLineSegment3DBasics, GeometryObject<FrameLineSegment3D>
{
   private ReferenceFrame referenceFrame;
   /** The line segment. */
   private final LineSegment3D lineSegment = new LineSegment3D();

   private final FixedFramePoint3DBasics firstEndpoint = new FixedFramePoint3DBasics()
   {
      @Override
      public void setX(double x)
      {
         lineSegment.getFirstEndpoint().setX(x);
      }

      @Override
      public void setY(double y)
      {
         lineSegment.getFirstEndpoint().setY(y);
      }

      @Override
      public void setZ(double z)
      {
         lineSegment.getFirstEndpoint().setZ(z);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return lineSegment.getFirstEndpointX();
      }

      @Override
      public double getY()
      {
         return lineSegment.getFirstEndpointY();
      }

      @Override
      public double getZ()
      {
         return lineSegment.getFirstEndpointZ();
      }
   };

   private final FixedFramePoint3DBasics secondEndpoint = new FixedFramePoint3DBasics()
   {
      @Override
      public void setX(double x)
      {
         lineSegment.getSecondEndpoint().setX(x);
      }

      @Override
      public void setY(double y)
      {
         lineSegment.getSecondEndpoint().setY(y);
      }

      @Override
      public void setZ(double z)
      {
         lineSegment.getSecondEndpoint().setZ(z);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return lineSegment.getSecondEndpointX();
      }

      @Override
      public double getY()
      {
         return lineSegment.getSecondEndpointY();
      }

      @Override
      public double getZ()
      {
         return lineSegment.getSecondEndpointZ();
      }
   };

   public FrameLineSegment3D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   public FrameLineSegment3D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameLineSegment3D(LineSegment3DReadOnly segment)
   {
      this(ReferenceFrame.getWorldFrame(), segment);
   }

   public FrameLineSegment3D(ReferenceFrame referenceFrame, LineSegment3DReadOnly segment)
   {
      setIncludingFrame(referenceFrame, segment);
   }

   public FrameLineSegment3D(FrameLineSegment3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   @Override
   public void set(FrameLineSegment3D other)
   {
      FrameLineSegment3DBasics.super.set(other);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   public FixedFramePoint3DBasics getFirstEndpoint()
   {
      return firstEndpoint;
   }

   @Override
   public FixedFramePoint3DBasics getSecondEndpoint()
   {
      return secondEndpoint;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Tests on a per-component basis on both endpoints if this line segment is equal to
    * {@code other} with the tolerance {@code epsilon}.
    * <p>
    * If the two line segments have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameLineSegment3D other, double epsilon)
   {
      return FrameLineSegment3DBasics.super.epsilonEquals(other, epsilon);
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
   public boolean geometricallyEquals(FrameLineSegment3D other, double epsilon)
   {
      return FrameLineSegment3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(FrameLineSegment3DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two line segments have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other line segment 3D to compare against this. Not modified.
    * @return {@code true} if the two line segments are exactly equal component-wise and are
    *         expressed in the same reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((FrameLineSegment3DReadOnly) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Provides a {@code String} representation of this frame line segment 3D as follows:<br>
    * Line segment 3D: 1st endpoint = ( 0.174, 0.732, -0.222 ), 2nd endpoint = (-0.558, -0.380,
    * 0.130 )-worldFrame
    *
    * @return the {@code String} representing this line segment 3D.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getLineSegment3DString(this) + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this line segment
    * 3D.
    *
    * @return the hash code value for this line 3D.
    */
   @Override
   public int hashCode()
   {
      return lineSegment.hashCode();
   }
}
