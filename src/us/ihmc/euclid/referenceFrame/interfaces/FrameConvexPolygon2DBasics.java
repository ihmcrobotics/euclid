package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface FrameConvexPolygon2DBasics extends FixedFrameConvexPolygon2DBasics
{
   /**
    * Sets the reference frame of this polygon without updating or modifying the coordinates of its
    * vertices.
    *
    * @param referenceFrame the new reference frame for this frame convex polygon.
    */
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Clears this polygon, adds a single vertex at (0, 0), and updates it.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      FixedFrameConvexPolygon2DBasics.super.setToZero();
   }

   /**
    * Clears this polygon, adds a single vertex at ({@link Double#NaN}, {@link Double#NaN}), and
    * updates it.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      FixedFrameConvexPolygon2DBasics.super.setToNaN();
   }
}
