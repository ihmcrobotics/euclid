package us.ihmc.euclid.geometry.exceptions;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * {@code RuntimeException} dedicated to improper 2D or 3D bounding boxes.
 *
 * @author Sylvain Bertrand
 *
 */
public class BoundingBoxException extends RuntimeException
{
   private static final long serialVersionUID = 6490958054490430093L;

   /**
    * Creates a new bounding box exception with no detail message.
    */
   public BoundingBoxException()
   {
   }

   /**
    * Creates a new bounding box exception with the specified detail message.
    *
    * @param message the detail message.
    */
   public BoundingBoxException(String message)
   {
      super(message);
   }

   /**
    * Creates a new bounding box exception with a default detail message containing the value of the
    * given {@code boundingBox2D}.
    *
    * @param badBoundingBox2D the bounding box 2D to be displayed in the detail message. Not modified.
    */
   public BoundingBoxException(BoundingBox2DReadOnly badBoundingBox2D)
   {
      super("Improper bounding box 2D: " + (badBoundingBox2D == null ? "bounding box is null" : badBoundingBox2D.toString()));
   }

   /**
    * Creates a new bounding box exception with a default detail message containing the value of the
    * given minimum and maximum coordinates of the bounding box.
    *
    * @param boundingBoxMin the minimum coordinate of the bounding box to be displayed in the detail
    *           message. Not modified.
    * @param boundingBoxMax the maximum coordinate of the bounding box to be displayed in the detail
    *           message. Not modified.
    */
   public BoundingBoxException(Point2DReadOnly boundingBoxMin, Point2DReadOnly boundingBoxMax)
   {
      super("Improper bounding box 2D: min = " + boundingBoxMin + ", max = " + boundingBoxMax);
   }

   /**
    * Creates a new bounding box exception with a default detail message containing the value of the
    * given {@code boundingBox3D}.
    *
    * @param badBoundingBox3D the bounding box 3D to be displayed in the detail message. Not modified.
    */
   public BoundingBoxException(BoundingBox3D badBoundingBox3D)
   {
      super("Improper bounding box 3D: " + (badBoundingBox3D == null ? "bounding box is null" : badBoundingBox3D.toString()));
   }

   /**
    * Creates a new bounding box exception with a default detail message containing the value of the
    * given minimum and maximum coordinates of the bounding box.
    *
    * @param boundingBoxMin the minimum coordinate of the bounding box to be displayed in the detail
    *           message. Not modified.
    * @param boundingBoxMax the maximum coordinate of the bounding box to be displayed in the detail
    *           message. Not modified.
    */
   public BoundingBoxException(Point3DReadOnly boundingBoxMin, Point3DReadOnly boundingBoxMax)
   {
      super("Improper bounding box 3D: min = " + boundingBoxMin + ", max = " + boundingBoxMax);
   }

   /**
    * Creates a new bounding box exception with a default detail message containing the value of the
    * given minimum and maximum coordinates of the bounding box.
    *
    * @param boundingBoxMinX the minimum x-coordinate of the bounding box to be displayed in the detail
    *           message.
    * @param boundingBoxMinY the minimum y-coordinate of the bounding box to be displayed in the detail
    *           message.
    * @param boundingBoxMinZ the minimum z-coordinate of the bounding box to be displayed in the detail
    *           message.
    * @param boundingBoxMaxX the maximum x-coordinate of the bounding box to be displayed in the detail
    *           message.
    * @param boundingBoxMaxY the maximum y-coordinate of the bounding box to be displayed in the detail
    *           message.
    * @param boundingBoxMaxZ the maximum z-coordinate of the bounding box to be displayed in the detail
    *           message.
    */
   public BoundingBoxException(double boundingBoxMinX, double boundingBoxMinY, double boundingBoxMinZ, double boundingBoxMaxX, double boundingBoxMaxY,
                               double boundingBoxMaxZ)
   {
      super("Improper bounding box 3D: min = " + EuclidCoreIOTools.getStringOf("(", " )", ", ", boundingBoxMinX, boundingBoxMinY, boundingBoxMinZ) + ", max = "
            + EuclidCoreIOTools.getStringOf("(", " )", ", ", boundingBoxMaxX, boundingBoxMaxY, boundingBoxMaxZ));
   }
}
