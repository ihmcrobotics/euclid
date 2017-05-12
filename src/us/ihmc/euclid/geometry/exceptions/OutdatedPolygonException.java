package us.ihmc.euclid.geometry.exceptions;

public class OutdatedPolygonException extends RuntimeException
{
   private static final long serialVersionUID = -5043468839061602341L;

   public OutdatedPolygonException(String description)
   {
      super(description);
   }
}