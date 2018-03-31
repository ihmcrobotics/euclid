package us.ihmc.euclid.referenceFrame.interfaces;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface FrameConvexPolygon2DBasics extends FixedFrameConvexPolygon2DBasics, FrameChangeable
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

   default void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      setReferenceFrame(referenceFrame);
      setAndUpdate(vertices, numberOfVertices);
   }

   default void setIncludingFrameAndUpdate3D(ReferenceFrame referenceFrame, List<? extends Point3DReadOnly> vertices, int numberOfVertices)
   {
      setReferenceFrame(referenceFrame);
      setAndUpdate3D(vertices, numberOfVertices);
   }

   default void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, Point2DReadOnly[] vertices, int numberOfVertices)
   {
      setReferenceFrame(referenceFrame);
      setAndUpdate(vertices, numberOfVertices);
   }

   default void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, Point3DReadOnly[] vertices, int numberOfVertices)
   {
      setReferenceFrame(referenceFrame);
      setAndUpdate(vertices, numberOfVertices);
   }

   default void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, double[][] vertices, int numberOfVertices)
   {
      setReferenceFrame(referenceFrame);
      setAndUpdate(vertices, numberOfVertices);
   }

   default void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, ConvexPolygon2DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      setAndUpdate(other);
   }

   default void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, ConvexPolygon2DReadOnly firstPolygon, ConvexPolygon2DReadOnly secondPolygon)
   {
      setReferenceFrame(referenceFrame);
      setAndUpdate(firstPolygon, secondPolygon);
   }

   default void setIncludingFrameAndUpdate(List<? extends FramePoint2DReadOnly> vertices, int numberOfVertices)
   {
      setReferenceFrame(vertices.get(0).getReferenceFrame());
      setAndUpdate(vertices, numberOfVertices);
   }

   default void setIncludingFrameAndUpdate3D(List<? extends FramePoint3DReadOnly> vertices, int numberOfVertices)
   {
      setReferenceFrame(vertices.get(0).getReferenceFrame());
      setAndUpdate3D(vertices, numberOfVertices);
   }

   default void setIncludingFrameAndUpdate(FramePoint2DReadOnly[] vertices, int numberOfVertices)
   {
      setReferenceFrame(vertices[0].getReferenceFrame());
      setAndUpdate(vertices, numberOfVertices);
   }

   default void setIncludingFrameAndUpdate(FramePoint3DReadOnly[] vertices, int numberOfVertices)
   {
      setReferenceFrame(vertices[0].getReferenceFrame());
      setAndUpdate(vertices, numberOfVertices);
   }

   default void setIncludingFrameAndUpdate(FrameConvexPolygon2DReadOnly other)
   {
      setIncludingFrameAndUpdate(other.getReferenceFrame(), other);
   }

   default void setIncludingFrameAndUpdate(FrameConvexPolygon2DReadOnly firstPolygon, FrameConvexPolygon2DReadOnly secondPolygon)
   {
      firstPolygon.checkReferenceFrameMatch(secondPolygon);
      setReferenceFrame(firstPolygon.getReferenceFrame());
      setAndUpdate((ConvexPolygon2DReadOnly) firstPolygon, (ConvexPolygon2DReadOnly) secondPolygon);
   }

   void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame);
}
