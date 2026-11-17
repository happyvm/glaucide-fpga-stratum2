# Outils de preuve formelle

## Vérificateur de contrats garantie/hypothèse (`contracts.py`)

Vérifie la méthode « garantie/hypothèse » de `docs/formal.md` par le texte des
messages : chaque `assume!` doit être une garantie (`check!` de même texte)
prouvée dans une autre unité, ou une hypothèse d'environnement déclarée.

```sh
python3 tools/formal/contracts.py          # rapport + code de sortie
python3 -m unittest discover tools/formal  # tests du vérificateur
```

Le script ne dépend que de la bibliothèque standard Python 3. Il lit
`rtl/src/*.rs` (sans les fichiers de test ni les fichiers générés) et ignore
les commentaires et les blocs `#[cfg(test)]`.

Règles :

- **Erreur** : un `assume!` dont le message n'est ni le message d'un `check!`
  d'une autre unité, ni une entrée de `environment_assumptions.txt`.
- **Avertissement** : un `assume!` dont le message est aussi un `check!` de la
  même unité (garantie circulaire) ;
- **Avertissement** : un même message utilisé pour des conditions différentes
  dans une unité ;
- **Pas d'erreur** : un `check!` qu'aucune autre unité n'assume.

Le code de sortie vaut 0 sans erreur, 1 avec erreur, 2 si un fichier ne se lit pas.

### Ajouter une hypothèse d'environnement

Une hypothèse qui n'a pas de garant dans le code (flux d'entrée de `stream.rs`,
hypothèse de configuration, environnement physique) s'ajoute à
`environment_assumptions.txt`, avec le message exact et une justification :

```text
message exact de l'assume! # garant ou raison, par exemple « stream contract, F0.5 »
```

Une justification « UNREVIEWED » produit un avertissement et doit être revue
par un humain. Une hypothèse qui a un `check!` de même texte dans un autre
module n'a pas besoin d'entrée : le vérificateur signale l'entrée inutile.
