package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * {@code FrameLineSegment2D} is a 2D line segment expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link LineSegment2DBasics}, a {@link ReferenceFrame} is associated
 * to a {@code FrameLineSegment2D}. This allows, for instance, to enforce, at runtime, that
 * operations on line segments occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a point in different
 * reference frames.
 * </p>
 * <p>
 * Because a {@code FrameLineSegment2D} extends {@code LineSegment2DBasics}, it is compatible with
 * methods only requiring {@code LineSegment2DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameLineSegment2D}.
 * </p>
 */
public class FrameLineSegment2D implements FrameLineSegment2DBasics, GeometryObject<FrameLineSegment2D>
{
   /** The reference frame in which this line is expressed. */
   private ReferenceFrame referenceFrame;
   /** The line segment. */
   private final LineSegment2D lineSegment = new LineSegment2D();
   /** Rigid-body transform used to perform garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   private final FixedFramePoint2DBasics firstEndpoint = new FixedFramePoint2DBasics()
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
   };

   private final FixedFramePoint2DBasics secondEndpoint = new FixedFramePoint2DBasics()
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
   };

   /**
    * Default constructor that initializes both endpoints of this line segment to zero and its
    * reference frame to {@code ReferenceFrame.getWorldFrame()}.
    */
   public FrameLineSegment2D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Default constructor that initializes both endpoints of this line segment to zero and its
    * reference frame to {@code referenceFrame}.
    *
    * @param referenceFrame the initial reference frame for this line segment.
    */
   public FrameLineSegment2D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new line segment and initializes it to be same as the given line segment.
    * <p>
    * The reference frame is initialized to {@code ReferenceFrame.getWorldFrame()}.
    * </p>
    *
    * @param lineSegment2DReadOnly the other line segment to copy. Not modified.
    */
   public FrameLineSegment2D(LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), lineSegment2DReadOnly);
   }

   /**
    * Creates a new line segment and initializes it to be same as the given line segment.
    *
    * @param referenceFrame the initial reference frame for this line segment.
    * @param lineSegment2DReadOnly the other line segment to copy. Not modified.
    */
   public FrameLineSegment2D(ReferenceFrame referenceFrame, LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      setIncludingFrame(referenceFrame, lineSegment2DReadOnly);
   }

   /**
    * Creates a new line segment and initializes it to be same as the given line segment.
    *
    * @param other the other line segment to copy. Not modified.
    */
   public FrameLineSegment2D(FrameLineSegment2DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Creates a new line segment and initializes to with the given endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *            frame.
    */
   public FrameLineSegment2D(FramePoint2DReadOnly firstEndpoint, FramePoint2DReadOnly secondEndpoint)
   {
      setIncludingFrame(firstEndpoint, secondEndpoint);
   }

   /** {@inheritDoc} */
   @Override
   public void set(FrameLineSegment2D other)
   {
      FrameLineSegment2DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getFirstEndpoint()
   {
      return firstEndpoint;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getSecondEndpoint()
   {
      return secondEndpoint;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      /*
       * By overriding changeFrame, on the transformToDesiredFrame is being checked instead of checking
       * both referenceFrame.transformToRoot and desiredFrame.transformToRoot.
       */
      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      referenceFrame = desiredFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame, false);
      referenceFrame = desiredFrame;
   }

   /**
    * Tests on a per-component basis on both endpoints if this line segment is equal to {@code other}
    * with the tolerance {@code epsilon}.
    * <p>
    * If the two line segments have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal and are expressed in the same reference
    *         frame, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameLineSegment2D other, double epsilon)
   {
      return FrameLineSegment2DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two lines are geometrically similar.
    * <p>
    * Two lines are considered geometrically equal is they are collinear, pointing toward the same or
    * opposite direction.
    * </p>
    *
    * @param other the line to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two lines represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   @Override
   public boolean geometricallyEquals(FrameLineSegment2D other, double epsilon)
   {
      return FrameLineSegment2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameLineSegment2DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two line segments have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if the two line segments are exactly equal component-wise and are expressed
    *         in the same reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((FrameLineSegment2DReadOnly) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Provides a {@code String} representation of this frame line segment 2D as follows:<br>
    * Line segment 2D: 1st endpoint = ( 0.174, 0.732, -0.222 ), 2nd endpoint = (-0.558, -0.380, 0.130
    * )-worldFrame
    *
    * @return the {@code String} representing this line segment 2D.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getLineSegment2DString(this) + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this line segment
    * 2D.
    *
    * @return the hash code value for this line segment 2D.
    */
   @Override
   public int hashCode()
   {
      return lineSegment.hashCode();
   }
}