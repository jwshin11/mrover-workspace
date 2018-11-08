class Searcher {
public:

  Searchstate run();

  Searcher();

private:
  enum class SearchState
  {
    SearchFaceNorth,
    SearchTurn120,
    SearchTurn240,
    SearchTurn360,
    SearchTurn,
    SearchDrive,
    TurnToBall,
    DriveToBall
  };

  SearchState executeSearchFaceNorth();

  SearchState executeSearchTurn120();

  SearchState executeSearchTurn240();

  SearchState executeSearchTurn360();

  SearchState executeSearchTurn();

  SearchState executeSearchDrive();

  SearchState executeTurnToBall();

  SearchState executeDriveToBall();

  SearchState currentState();
};
