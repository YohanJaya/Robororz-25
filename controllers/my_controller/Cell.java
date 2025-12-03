import java.util.Objects;

public class Cell {
    int x;
    int y;
    Cell parent;

    public Cell(int x, int y) {
      this.x = x;
      this.y = y;
      this.parent = null;
    }

    public Cell(int x, int y, Cell parent) {
      this.x = x;
      this.y = y;
      this.parent = parent;
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) return true;
      if (o == null || getClass() != o.getClass()) return false;
      Cell cell = (Cell) o;
      return x == cell.x && y == cell.y;
    }

    @Override
    public int hashCode() {
      return Objects.hash(x, y);
    }
}
